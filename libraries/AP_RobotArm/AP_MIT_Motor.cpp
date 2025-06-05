#include "AP_MIT_Motor.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_CANManager/AP_CANManager.h>

extern const AP_HAL::HAL& hal;

// Control modes
#define CTRL_MODE_POSITION 0
#define CTRL_MODE_VELOCITY 1  
#define CTRL_MODE_CURRENT 2

// Command IDs
#define CMD_GET_CURRENT   0x04
#define CMD_GET_VELOCITY  0x06
#define CMD_GET_POSITION  0x08
#define CMD_GET_TEMP      0x32
#define CMD_SET_ID        0x01
#define CMD_SAVE_FLASH    0x0E

// Control types
#define POSITION 0
#define SPEED    1
#define CURRENT  2
#define SET_ID   3

// 构造函数
MIT_Motor::MIT_Motor() : _can_stats() {
    // 初始化统计信息
}

void MIT_Motor::handle_motor_control(MotorInstance* motor)
{
    // Check for null pointer and enabled status
    if (!motor || !motor->enabled) {
        return;
    }

    // Handle different control modes
    switch (motor->mode) {
        case CTRL_MODE_POSITION:
            tihu_motor_ctrl(motor->can_id, motor->motor_id, 
                          POSITION, motor->target_value);
            break;
            
        case CTRL_MODE_VELOCITY:
            tihu_motor_ctrl(motor->can_id, motor->motor_id,
                          SPEED, motor->target_value);
            break;
            
        case CTRL_MODE_CURRENT:
            tihu_motor_ctrl(motor->can_id, motor->motor_id,
                          CURRENT, motor->target_value);
            break;
            
        default:
            // Log error using ArduPilot's logging system
            AP::logger().Write_MessageF("MIT_Motor: Invalid control mode: %d", motor->mode);
            break;
    }
    
    // Get motor status information
    tihu_motor_one_byte_ctrl(motor->can_id, motor->motor_id, CMD_GET_CURRENT);  // get current
    tihu_motor_one_byte_ctrl(motor->can_id, motor->motor_id, CMD_GET_VELOCITY); // get velocity
    tihu_motor_one_byte_ctrl(motor->can_id, motor->motor_id, CMD_GET_POSITION); // get position
    tihu_motor_one_byte_ctrl(motor->can_id, motor->motor_id, CMD_GET_TEMP);     // get temperature
}

void MIT_Motor::tihu_motor_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_type, float target_value)
{
    AP_HAL::CANFrame txmsg {};
    
    txmsg.id = can_id;
    txmsg.dlc = 5;  // Data Length Code = 5 bytes
    
    // First byte is control type
    txmsg.data[0] = ctrl_type;
    
    // Convert float target value to int32 and split into bytes
    int32_t value = static_cast<int32_t>(target_value);
    txmsg.data[1] = (value >> 24) & 0xFF;
    txmsg.data[2] = (value >> 16) & 0xFF;
    txmsg.data[3] = (value >> 8) & 0xFF;
    txmsg.data[4] = value & 0xFF;
    
    // Log the CAN message
    AP::logger().Write_MessageF("MIT_Motor: CAN TX ID=0x%02X type=%d value=%d", 
                               (unsigned)can_id, (unsigned)ctrl_type, (int)value);
    
    // Send the frame with 10ms timeout
    write_frame(txmsg, 10 * 1000);
}

void MIT_Motor::tihu_motor_one_byte_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t cmd)
{
    AP_HAL::CANFrame txmsg {};
    
    txmsg.id = can_id;
    txmsg.dlc = 1;  // Single byte command
    txmsg.data[0] = cmd;
    
    // Log the command
    AP::logger().Write_MessageF("MIT_Motor: CAN TX ID=0x%02X cmd=0x%02X", 
                               (unsigned)can_id, (unsigned)cmd);
    
    // Send the frame with 10ms timeout
    write_frame(txmsg, 10 * 1000);
}

bool MIT_Motor::write_frame(AP_HAL::CANFrame& frame, uint64_t timeout_us)
{
    // 记录发送的CAN消息到日志
    AP::logger().Write_MessageF("MIT Motor TX: ID=0x%02X DLC=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]", 
                               (unsigned)frame.id, (unsigned)frame.dlc,
                               (unsigned)frame.data[0], (unsigned)frame.data[1],
                               (unsigned)frame.data[2], (unsigned)frame.data[3],
                               (unsigned)frame.data[4], (unsigned)frame.data[5],
                               (unsigned)frame.data[6], (unsigned)frame.data[7]);
                               
    // AP_RobotArm模块专门通过CAN1发送，优先尝试DroneCAN方式
    if (try_send_via_dronecan(frame, timeout_us)) {
        return true;
    }
    
    // 如果DroneCAN不可用，尝试直接通过HAL发送到CAN1 (index=0)
    return try_send_via_hal_can1(frame, timeout_us);
}

bool MIT_Motor::try_send_via_dronecan(AP_HAL::CANFrame& frame, uint64_t timeout_us)
{
    AP_DroneCAN* dronecan = nullptr;
    
    // 尝试获取第一个可用的DroneCAN驱动实例
    for (uint8_t i = 0; i < HAL_MAX_CAN_PROTOCOL_DRIVERS; i++) {
        dronecan = AP_DroneCAN::get_dronecan(i);
        if (dronecan != nullptr) {
            break;
        }
    }
    
    if (dronecan == nullptr) {
        AP::logger().Write_MessageF("MIT_Motor: No DroneCAN driver available");
        return false;
    }
    
    // write_aux_frame只支持11位标准CAN ID
    if (frame.isExtended()) {
        AP::logger().Write_MessageF("MIT_Motor: Extended CAN ID 0x%08X not supported by DroneCAN aux", 
                                   (unsigned)frame.id);
        _can_stats.dronecan_failures++;
        return false;
    }
    
    // 记录详细的发送信息
    AP::logger().Write_MessageF("MIT_Motor: TX via DroneCAN[%d] ID=0x%02X DLC=%d", 
                               dronecan->get_driver_index(), 
                               (unsigned)frame.id, 
                               (unsigned)frame.dlc);
    
    bool result = dronecan->write_aux_frame(frame, timeout_us);
    if (!result) {
        AP::logger().Write_MessageF("MIT_Motor: DroneCAN send failed");
        _can_stats.dronecan_failures++;
        return false;
    }
    
    return true;
}

bool MIT_Motor::try_send_via_hal(AP_HAL::CANFrame& frame, uint64_t timeout_us)
{
    // 尝试多个CAN接口
    for (uint8_t iface_idx = 0; iface_idx < HAL_NUM_CAN_IFACES; iface_idx++) {
        AP_HAL::CANIface* iface = hal.can[iface_idx];
        if (iface != nullptr && iface->is_initialized()) {
            AP::logger().Write_MessageF("MIT_Motor: TX via HAL CAN%d ID=0x%02X DLC=%d", 
                                       iface_idx, 
                                       (unsigned)frame.id, 
                                       (unsigned)frame.dlc);
            
            int16_t result = iface->send(frame, timeout_us, AP_HAL::CANIface::AbortOnError);
            if (result > 0) {
                return true;  // 成功发送
            } else {
                AP::logger().Write_MessageF("MIT_Motor: CAN%d send failed, result=%d", 
                                           iface_idx, (int)result);
            }
        }
    }
    
    // 所有HAL接口都失败
    _can_stats.hal_failures++;
    return false;
}

bool MIT_Motor::try_send_via_hal_can1(AP_HAL::CANFrame& frame, uint64_t timeout_us)
{
    // 尝试直接通过HAL发送到CAN1 (index=0)
    AP_HAL::CANIface* iface = hal.can[0];
    if (iface != nullptr && iface->is_initialized()) {
        AP::logger().Write_MessageF("MIT_Motor: TX via HAL CAN1 ID=0x%02X DLC=%d", 
                                   (unsigned)frame.id, 
                                   (unsigned)frame.dlc);
        
        int16_t result = iface->send(frame, timeout_us, AP_HAL::CANIface::AbortOnError);
        if (result > 0) {
            return true;  // 成功发送
        } else {
            AP::logger().Write_MessageF("MIT_Motor: CAN1 send failed, result=%d", 
                                       (int)result);
        }
    }
    
    // 所有HAL接口都失败
    _can_stats.hal_failures++;
    return false;
}

void MIT_Motor::print_can_stats() const
{
    AP::logger().Write_MessageF("MIT_Motor CAN Stats - Total:%u DroneCAN(S:%u/F:%u) HAL(S:%u/F:%u)", 
                               (unsigned)_can_stats.total_frames,
                               (unsigned)_can_stats.dronecan_success,
                               (unsigned)_can_stats.dronecan_failures,
                               (unsigned)_can_stats.hal_success,
                               (unsigned)_can_stats.hal_failures);
}

bool MIT_Motor::test_can_send(uint8_t can_id, uint8_t test_cmd)
{
    AP::logger().Write_MessageF("MIT_Motor: Testing CAN send to ID=0x%02X cmd=0x%02X", 
                               (unsigned)can_id, (unsigned)test_cmd);
    
    AP_HAL::CANFrame test_frame {};
    test_frame.id = can_id;
    test_frame.dlc = 1;
    test_frame.data[0] = test_cmd;  // 默认使用"获取电机状态"命令
    
    bool result = write_frame(test_frame, 10000);  // 10ms timeout
    
    // 打印测试结果和当前统计信息
    AP::logger().Write_MessageF("MIT_Motor: Test CAN send %s", result ? "SUCCESS" : "FAILED");
    print_can_stats();
    
    return result;
}
