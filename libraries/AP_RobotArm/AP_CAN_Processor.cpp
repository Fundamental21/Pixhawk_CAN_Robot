#include "AP_CAN_Processor.h"
#include "../AP_WheelEncoder/AP_Hall_Can_Backend.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <cmath>
#include <algorithm>
#include <memory>
#include <functional>
#include <limits>

namespace RobotArm {

// Global motor data instances with cache-line alignment
alignas(32) MitVelocityData mit_receive_velocity{};
alignas(32) MitPositionData mit_receive_position{};
alignas(32) MitCurrentData mit_receive_current{};
alignas(32) MitTemperatureData mit_receive_temperature{};
alignas(32) MitStateData mit_receive_state{};

// Optimized valid one-byte commands array - sorted for binary search
const std::array<uint16_t, 14> VALID_ONE_BYTE_CMDS = {{
    0x02,  // stop motor
    0x03,  // get motor state
    0x04,  // get motor current
    0x05,  // get motor target current
    0x06,  // get motor speed
    0x07,  // get motor target speed
    0x08,  // get motor position
    0x14,  // get bus voltage
    0x17,  // get max accel
    0x18,  // get max velocity
    0x31,  // get motor temperature
    0x32,  // get board temperature
    0x54,  // get encoder offset
    0x0E   // save to flash
}};

// Optimized parser configurations with fast lookup
const std::array<MotorDataParserConfig, PARSER_CONFIG_COUNT> parser_configs = {{
    {0x04, 1.0f / 1000.0f, &mit_receive_current},
    {0x06, 0.6f / 101.0f, &mit_receive_velocity},
    {0x08, 360.0f / 262144.0f, &mit_receive_position},
    {0x14, 1.0f, &mit_receive_state},
    {0x32, 1.0f, &mit_receive_temperature}
}};

// Debug data with alignment
alignas(8) std::array<uint8_t, DEBUG_DATA_SIZE> debug_data{};

// 静态成员初始化
AP_CAN_Processor* AP_CAN_Processor::_singleton = nullptr;

AP_CAN_Processor::AP_CAN_Processor() : can_queue(nullptr) {
    if (_singleton == nullptr) {
        _singleton = this;
    }
}

AP_CAN_Processor::~AP_CAN_Processor() {
    if (_singleton == this) {
        _singleton = nullptr;
    }
}

bool AP_CAN_Processor::init(ObjectBuffer<CAN_Frame_Item>* queue) {
    if (queue == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AP_CAN_Processor: 队列指针为空");
        return false;
    }
    
    can_queue = queue;
    stats = Statistics(); // 重置统计信息
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_CAN_Processor: 初始化成功，支持CAN1/CAN2区分");
    AP::logger().Write_MessageF("AP_CAN_Processor: 初始化完成，队列容量=%u", 200);
    
    return true;
}

void AP_CAN_Processor::update() {
    if (can_queue == nullptr) {
        return;
    }
    
    uint64_t start_time = AP_HAL::micros64();
    const uint64_t max_process_time_us = 500; // 最大处理时间500μs，确保实时性
    uint8_t frames_processed = 0;
    const uint8_t max_frames_per_call = 20; // 每次调用最多处理20帧
    
    CAN_Frame_Item frame_item;
    while (frames_processed < max_frames_per_call && 
           can_queue->pop(frame_item) && 
           (AP_HAL::micros64() - start_time) < max_process_time_us) {
        
        // 基本验证
        if (frame_item.frame.dlc < 1 || frame_item.frame.dlc > 8) {
            stats.invalid_frames++;
            frames_processed++;
            continue;
        }
        
        uint32_t can_id = frame_item.frame.id;
        uint8_t motor_id = can_id & 0xFF;
        
        // 验证电机ID范围 (1-8)
        if (motor_id < 1 || motor_id > 8) {
            stats.filtered_frames++;
            frames_processed++;
            continue;
        }
        
        // 处理CAN消息并更新统计信息
        if (process_can_frame(frame_item.frame, frame_item.can_bus_id)) {
            stats.processed_frames++;
            
            // 每1000帧记录一次详细统计（区分CAN总线）
            if ((stats.processed_frames % 1000) == 0) {
                AP::logger().Write_MessageF("CAN%d Motor%d: Processed=%u Invalid=%u Filtered=%u", 
                                          (int)frame_item.can_bus_id, (int)motor_id,
                                          (unsigned)stats.processed_frames, 
                                          (unsigned)stats.invalid_frames,
                                          (unsigned)stats.filtered_frames);
            }
        } else {
            stats.processing_errors++;
        }
        
        frames_processed++;
    }
    
    // 更新性能统计
    stats.update_calls++;
    uint64_t process_time = AP_HAL::micros64() - start_time;
    if (process_time > stats.max_process_time_us) {
        stats.max_process_time_us = process_time;
    }
}

bool AP_CAN_Processor::process_can_frame(const AP_HAL::CANFrame& frame, uint8_t can_bus_id) {
    // 复制到调试数据缓冲区
    const size_t copy_len = std::min(static_cast<size_t>(frame.dlc), DEBUG_DATA_SIZE);
    std::copy_n(frame.data, copy_len, debug_data.begin());

    const uint8_t cmd = frame.data[0];
    const uint32_t can_id = frame.id & AP_HAL::CANFrame::MaskStdID;
    
    // 验证CAN ID范围 (1-8)
    if (can_id < 1 || can_id > 8) {
        return false;
    }
    
    // 解析原始数据值 (data[1-4])，处理DLC不足情况
    int32_t raw_value = 0;
    if (frame.dlc >= 5) {
        raw_value = (frame.data[1] << 0)  | 
                   (frame.data[2] << 8)  | 
                   (frame.data[3] << 16) | 
                   (frame.data[4] << 24);
    } else {
        // DLC不足时，只使用可用字节
        for (uint8_t i = 1; i < frame.dlc; i++) {
            raw_value |= (frame.data[i] << ((i-1) * 8));
        }
    }
    
    // 确定通道：CAN总线ID到通道的映射
    const CanChannel channel = (can_bus_id == 1) ? CanChannel::CHANNEL_1 : CanChannel::CHANNEL_2;
    
    // 使用优化的解析器配置进行数据解析
    for (const auto& cfg : parser_configs) {
        if (cmd == cfg.cmd) {
            // 应用比例因子
            const float value = static_cast<float>(raw_value) * cfg.scale_factor;
            
            // 根据数据结构指针确定数据类型并更新相应的全局变量
            float* target = getTargetField(can_id, channel, cfg.data_struct_ptr);
            if (target) {
                *target = value;
                return true;
            }
        }
    }
    
    return false; // 未找到匹配的命令
}

float* AP_CAN_Processor::getTargetField(uint32_t can_id, CanChannel channel, void* struct_ptr) noexcept {
    if (!struct_ptr || can_id == 0 || can_id > 8) {
        return nullptr;
    }

    const size_t index = can_id - 1;
    if (index >= MAX_MOTORS_PER_CHANNEL) {
        return nullptr;
    }

    // 类型安全的数据结构访问
    if (struct_ptr == &mit_receive_velocity) {
        return mit_receive_velocity.get_safe(channel, index);
    } else if (struct_ptr == &mit_receive_position) {
        return mit_receive_position.get_safe(channel, index);
    } else if (struct_ptr == &mit_receive_current) {
        return mit_receive_current.get_safe(channel, index);
    } else if (struct_ptr == &mit_receive_temperature) {
        return mit_receive_temperature.get_safe(channel, index);
    } else if (struct_ptr == &mit_receive_state) {
        return mit_receive_state.get_safe(channel, index);
    }
    
    return nullptr;
}

// ==================== TihuMotorController 实现 ====================

// 快速命令验证使用二分搜索
bool TihuMotorController::isValidCtrlCmd(uint16_t cmd) noexcept {
    return std::binary_search(VALID_ONE_BYTE_CMDS.begin(), VALID_ONE_BYTE_CMDS.end(), cmd);
}

// 优化的值转换函数
int32_t TihuMotorController::convertSpeed(float value) noexcept {
    return static_cast<int32_t>(value * 101.0f * (5.0f / 3.0f));
}

int32_t TihuMotorController::convertPosition(float value) noexcept {
    return static_cast<int32_t>(value / 360.0f * 262144.0f);
}

int32_t TihuMotorController::convertCurrent(float value) noexcept {
    return static_cast<int32_t>(value * 1000.0f);
}

// 使用ArduPilot的CAN接口发送命令
bool TihuMotorController::sendCommand(uint8_t can_id, uint8_t motor_id, uint8_t cmd, int32_t value) noexcept {
    const uint32_t raw_value = static_cast<uint32_t>(value);
    
    // 创建CAN帧
    AP_HAL::CANFrame frame;
    frame.id = motor_id;
    frame.dlc = 5;
    frame.data[0] = cmd;
    frame.data[1] = static_cast<uint8_t>(raw_value >> 0);
    frame.data[2] = static_cast<uint8_t>(raw_value >> 8);
    frame.data[3] = static_cast<uint8_t>(raw_value >> 16);
    frame.data[4] = static_cast<uint8_t>(raw_value >> 24);

    // 通过Hall_Can_Backend发送（或者直接使用ArduPilot CAN接口）
    Hall_Can_Backend* hall_backend = Hall_Can_Backend::get_singleton();
    if (hall_backend) {
        // 记录发送的CAN帧
        hall_backend->Log_Write_CAN_TX(frame);
        // 这里需要实现实际的CAN发送逻辑，使用ArduPilot的CAN接口
        // TODO: 集成ArduPilot的CAN发送接口
        return true;
    }
    
    return false;
}

void TihuMotorController::motorCtrl(uint8_t can_id, uint8_t motor_id, MotorCtrlMode mode, float value) noexcept {
    // 快速验证
    if (!isValidCanId(can_id) || !isValidMotorId(motor_id) || !isValidCtrlMode(mode)) {
        return;
    }

    // 命令映射
    uint8_t cmd = 0;
    switch (mode) {
        case MotorCtrlMode::SPEED:    cmd = 0x1D; break;
        case MotorCtrlMode::POSITION: cmd = 0x1E; break;
        case MotorCtrlMode::CURRENT:  cmd = 0x1C; break;
        case MotorCtrlMode::SET_ID:   cmd = 0x2E; break;
        case MotorCtrlMode::SET_MAX_SPD: cmd = 0x24; break;
        case MotorCtrlMode::SET_MIN_SPD: cmd = 0x25; break;
        case MotorCtrlMode::SET_ZERO: return; // 不实现
        default: return;
    }

    // 快速值转换
    int32_t converted_value;
    switch (mode) {
        case MotorCtrlMode::SPEED:
            converted_value = convertSpeed(value);
            break;
        case MotorCtrlMode::POSITION:
            converted_value = convertPosition(value);
            break;
        case MotorCtrlMode::CURRENT:
            converted_value = convertCurrent(value);
            break;
        default:
            converted_value = static_cast<int32_t>(value);
            break;
    }
    
    sendCommand(can_id, motor_id, cmd, converted_value);
}

void TihuMotorController::motorOneByteCtrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_cmd) noexcept {
    if (!isValidCanId(can_id) || !isValidMotorId(motor_id) || !isValidCtrlCmd(ctrl_cmd)) {
        return;
    }

    // 创建单字节CAN帧
    AP_HAL::CANFrame frame;
    frame.id = motor_id;
    frame.dlc = 1;
    frame.data[0] = ctrl_cmd;

    // 通过Hall_Can_Backend发送
    Hall_Can_Backend* hall_backend = Hall_Can_Backend::get_singleton();
    if (hall_backend) {
        hall_backend->Log_Write_CAN_TX(frame);
        // TODO: 实现实际的CAN发送
    }
}

float TihuMotorController::getMotorPosition(uint8_t can_id, uint8_t motor_id) noexcept {
    if (!isValidCanId(can_id) || !isValidMotorId(motor_id)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    
    const CanChannel channel = canIdToChannel(can_id);
    const uint8_t index = motorIdToIndex(motor_id);
    
    const float* pos = mit_receive_position.get_safe(channel, index);
    return pos ? *pos : std::numeric_limits<float>::quiet_NaN();
}

float TihuMotorController::getMotorVelocity(uint8_t can_id, uint8_t motor_id) noexcept {
    if (!isValidCanId(can_id) || !isValidMotorId(motor_id)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    
    const CanChannel channel = canIdToChannel(can_id);
    const uint8_t index = motorIdToIndex(motor_id);
    
    const float* vel = mit_receive_velocity.get_safe(channel, index);
    return vel ? *vel : std::numeric_limits<float>::quiet_NaN();
}

float TihuMotorController::getMotorCurrent(uint8_t can_id, uint8_t motor_id) noexcept {
    if (!isValidCanId(can_id) || !isValidMotorId(motor_id)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    
    const CanChannel channel = canIdToChannel(can_id);
    const uint8_t index = motorIdToIndex(motor_id);
    
    const float* current = mit_receive_current.get_safe(channel, index);
    return current ? *current : std::numeric_limits<float>::quiet_NaN();
}

float TihuMotorController::getMotorTemperature(uint8_t can_id, uint8_t motor_id) noexcept {
    if (!isValidCanId(can_id) || !isValidMotorId(motor_id)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    
    const CanChannel channel = canIdToChannel(can_id);
    const uint8_t index = motorIdToIndex(motor_id);
    
    const float* temp = mit_receive_temperature.get_safe(channel, index);
    return temp ? *temp : std::numeric_limits<float>::quiet_NaN();
}

} // namespace RobotArm 