#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <cstdlib>
#include <cstdint>
#include <array>

// Forward declaration
class Hall_Can_Backend;

namespace RobotArm {

// Constants - more specific types for better optimization
constexpr size_t PARSER_CONFIG_COUNT = 5;
constexpr size_t MOTOR_ID_MAP_COUNT = 16;
constexpr size_t DEBUG_DATA_SIZE = 8;
constexpr size_t MAX_MOTORS_PER_CHANNEL = 8;
constexpr uint8_t INVALID_MOTOR_INDEX = 0xFF;

// Enums with explicit underlying types for better performance
enum class MotorCtrlMode : uint8_t {
    SPEED = 0,
    POSITION = 1,
    CURRENT = 2,
    SET_ID = 3,
    SET_ZERO = 4,
    SET_MAX_SPD = 5,
    SET_MIN_SPD = 6
};

enum class CanChannel : uint8_t {
    CHANNEL_1 = 0,
    CHANNEL_2 = 1
};

// Optimized structures with better memory layout
struct MotorDataParserConfig {
    uint8_t cmd;
    float scale_factor;
    void* data_struct_ptr;
    
    constexpr MotorDataParserConfig(uint8_t c, float sf, void* ptr) noexcept
        : cmd(c), scale_factor(sf), data_struct_ptr(ptr) {}
};

// Optimized template with better memory layout and no exceptions
template<typename T>
struct MotorChannelData {
    alignas(32) std::array<T, MAX_MOTORS_PER_CHANNEL> can1; // Cache-line aligned
    alignas(32) std::array<T, MAX_MOTORS_PER_CHANNEL> can2;
    
    // 构造函数：初始化所有数组元素为默认值
    MotorChannelData() {
        can1.fill(T{});
        can2.fill(T{});
    }
    
    // Fast array access without exceptions (for embedded systems)
    inline T& get_unsafe(CanChannel channel, size_t index) noexcept {
        return (channel == CanChannel::CHANNEL_1) ? can1[index] : can2[index];
    }
    
    inline const T& get_unsafe(CanChannel channel, size_t index) const noexcept {
        return (channel == CanChannel::CHANNEL_1) ? can1[index] : can2[index];
    }
    
    // Safe access with bounds checking (returns pointer for better performance)
    inline T* get_safe(CanChannel channel, size_t index) noexcept {
        if (index >= MAX_MOTORS_PER_CHANNEL) {
            return nullptr;
        }
        return &((channel == CanChannel::CHANNEL_1) ? can1[index] : can2[index]);
    }
    
    inline const T* get_safe(CanChannel channel, size_t index) const noexcept {
        if (index >= MAX_MOTORS_PER_CHANNEL) {
            return nullptr;
        }
        return &((channel == CanChannel::CHANNEL_1) ? can1[index] : can2[index]);
    }
};

// Type aliases for motor data
using MitVelocityData = MotorChannelData<float>;
using MitPositionData = MotorChannelData<float>;
using MitCurrentData = MotorChannelData<float>;
using MitTemperatureData = MotorChannelData<float>;
using MitStateData = MotorChannelData<float>;

// External data declarations - aligned for better cache performance
extern alignas(32) MitVelocityData mit_receive_velocity;
extern alignas(32) MitPositionData mit_receive_position;
extern alignas(32) MitCurrentData mit_receive_current;
extern alignas(32) MitTemperatureData mit_receive_temperature;
extern alignas(32) MitStateData mit_receive_state;

// Fast lookup tables for performance
extern const std::array<MotorDataParserConfig, PARSER_CONFIG_COUNT> parser_configs;
extern const std::array<uint16_t, 14> VALID_ONE_BYTE_CMDS;

/**
 * @brief AP_RobotArm的CAN消息处理器
 * 
 * 这个类负责从Hall_Can_Backend的队列中取出CAN帧，
 * 并调用现有的AP_CAN_Receive处理逻辑进行电机数据解算
 * 
 * 设计要点：
 * - 支持100Hz的CAN消息处理频率
 * - 支持CAN1和CAN2总线区分
 * - 使用非阻塞队列避免实时性问题
 * - 复用现有的parser_configs和数据结构
 * - 提供详细的统计信息用于调试
 * - 合并了TihuMotorController的所有功能
 */
class AP_CAN_Processor {
public:
    // CAN帧队列项结构，包含CAN总线信息
    struct CAN_Frame_Item {
        AP_HAL::CANFrame frame;
        uint64_t timestamp_us;
        uint8_t can_bus_id;  // CAN总线ID: 1=CAN1, 2=CAN2
        
        CAN_Frame_Item() : timestamp_us(0), can_bus_id(0) {}
        CAN_Frame_Item(const AP_HAL::CANFrame& f, uint64_t ts, uint8_t bus_id) 
            : frame(f), timestamp_us(ts), can_bus_id(bus_id) {}
    };
    
    // 统计信息结构
    struct Statistics {
        uint32_t processed_frames;    // 成功处理的帧数
        uint32_t invalid_frames;      // 无效帧数（DLC错误等）
        uint32_t filtered_frames;     // 被过滤的帧数（非电机ID 1-8）
        uint32_t processing_errors;   // 处理错误数
        uint32_t update_calls;        // update调用次数
        uint64_t max_process_time_us; // 最大处理时间（微秒）
        
        Statistics() : processed_frames(0), invalid_frames(0), filtered_frames(0),
                      processing_errors(0), update_calls(0), max_process_time_us(0) {}
    };

    // 禁止拷贝
    CLASS_NO_COPY(AP_CAN_Processor);

    /**
     * @brief 获取单例实例
     */
    static AP_CAN_Processor* get_singleton() { return _singleton; }

    /**
     * @brief 初始化CAN处理器
     * @param queue CAN帧队列指针
     * @return true if successful, false otherwise
     */
    bool init(ObjectBuffer<CAN_Frame_Item>* queue);

    /**
     * @brief 更新函数 - 从队列处理CAN帧
     * 应该在主循环中定期调用（建议100Hz）
     * 
     * 每次调用会处理队列中的多个CAN帧，但限制处理时间
     * 以确保不影响系统的实时性能
     */
    void update();

    /**
     * @brief 获取处理统计信息
     */
    const Statistics& get_statistics() const { return stats; }

    /**
     * @brief 重置统计信息
     */
    void reset_statistics() { stats = Statistics(); }

private:
    static AP_CAN_Processor* _singleton;
    
    ObjectBuffer<CAN_Frame_Item>* can_queue;
    Statistics stats;
    
    // 私有构造函数（单例模式）
    AP_CAN_Processor();
    ~AP_CAN_Processor();
    
    // CAN帧处理函数
    bool process_can_frame(const AP_HAL::CANFrame& frame, uint8_t can_bus_id);
    
    // 目标字段获取函数
    float* getTargetField(uint32_t can_id, CanChannel channel, void* struct_ptr) noexcept;
};

/**
 * @brief 优化的电机控制类，集成到AP_CAN_Processor中
 * 
 * 提供完整的电机控制接口，使用ArduPilot的CAN系统
 */
class TihuMotorController {
public:
    // Static methods for motor control - marked inline for better performance
    static void motorCtrl(uint8_t can_id, uint8_t motor_id, MotorCtrlMode mode, float value) noexcept;
    static void motorOneByteCtrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_cmd) noexcept;
    static float getMotorPosition(uint8_t can_id, uint8_t motor_id) noexcept;
    static float getMotorVelocity(uint8_t can_id, uint8_t motor_id) noexcept;
    static float getMotorCurrent(uint8_t can_id, uint8_t motor_id) noexcept;
    static float getMotorTemperature(uint8_t can_id, uint8_t motor_id) noexcept;
    
    // Fast validation methods - constexpr and inline
    static constexpr bool isValidCanId(uint8_t can_id) noexcept {
        return (can_id == 1) || (can_id == 2);
    }
    
    static constexpr bool isValidMotorId(uint8_t motor_id) noexcept {
        return (motor_id > 0) && (motor_id <= MAX_MOTORS_PER_CHANNEL);
    }
    
    static constexpr bool isValidCtrlMode(MotorCtrlMode mode) noexcept {
        return static_cast<uint8_t>(mode) <= static_cast<uint8_t>(MotorCtrlMode::SET_MIN_SPD);
    }
    
    // Fast CAN ID to channel conversion
    static constexpr CanChannel canIdToChannel(uint8_t can_id) noexcept {
        return (can_id == 1) ? CanChannel::CHANNEL_1 : CanChannel::CHANNEL_2;
    }
    
    // Fast motor ID to index conversion (1-based to 0-based)
    static constexpr uint8_t motorIdToIndex(uint8_t motor_id) noexcept {
        return motor_id - 1;
    }

private:
    // 内部辅助函数
    static bool isValidCtrlCmd(uint16_t cmd) noexcept;
    static int32_t convertSpeed(float value) noexcept;
    static int32_t convertPosition(float value) noexcept;
    static int32_t convertCurrent(float value) noexcept;
    static bool sendCommand(uint8_t can_id, uint8_t motor_id, uint8_t cmd, int32_t value) noexcept;
};

// Inline utility functions for better performance
namespace detail {
    inline constexpr int32_t float_to_speed_raw(float value) noexcept {
        return static_cast<int32_t>(value * 101.0f * (5.0f / 3.0f));
    }
    
    inline constexpr int32_t float_to_position_raw(float value) noexcept {
        return static_cast<int32_t>(value / 360.0f * 262144.0f);
    }
    
    inline constexpr int32_t float_to_current_raw(float value) noexcept {
        return static_cast<int32_t>(value * 1000.0f);
    }
    
    inline constexpr float raw_to_position(int32_t raw_value) noexcept {
        return static_cast<float>(raw_value) * (360.0f / 262144.0f);
    }
    
    inline constexpr float raw_to_velocity(int32_t raw_value) noexcept {
        return static_cast<float>(raw_value) * (0.6f / 101.0f);
    }
    
    inline constexpr float raw_to_current(int32_t raw_value) noexcept {
        return static_cast<float>(raw_value) * (1.0f / 1000.0f);
    }
}

} // namespace RobotArm 