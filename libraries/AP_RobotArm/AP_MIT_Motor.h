#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/CANIface.h>

// 前向声明
class AP_DroneCAN;

class MIT_Motor {
public:
    // Motor instance structure
    struct MotorInstance {
        bool enabled;        // Whether the motor is enabled
        uint8_t mode;       // Control mode (position, velocity, current)
        uint8_t can_id;     // CAN ID of the motor
        uint8_t motor_id;   // Motor ID
        float target_value; // Target value for control
    };

    // CAN发送统计结构
    struct CANStats {
        uint32_t dronecan_success;    // DroneCAN发送成功次数
        uint32_t dronecan_failures;   // DroneCAN发送失败次数
        uint32_t hal_success;         // HAL发送成功次数
        uint32_t hal_failures;        // HAL发送失败次数
        uint32_t total_frames;        // 总发送帧数
        
        CANStats() : dronecan_success(0), dronecan_failures(0), 
                    hal_success(0), hal_failures(0), total_frames(0) {}
    };

    // 构造函数
    MIT_Motor();

    // Main motor control handler
    void handle_motor_control(MotorInstance* motor);
    
    // 获取CAN发送统计信息
    const CANStats& get_can_stats() const { return _can_stats; }
    
    // 重置统计信息
    void reset_can_stats() { _can_stats = CANStats(); }
    
    // 打印CAN发送统计信息
    void print_can_stats() const;
    
    // 测试CAN发送功能
    bool test_can_send(uint8_t can_id, uint8_t test_cmd = 0x03);

private:
    // Send control command to motor
    void tihu_motor_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_type, float target_value);
    
    // Send single byte command to motor
    void tihu_motor_one_byte_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t cmd);
    
    // Write CAN frame
    bool write_frame(AP_HAL::CANFrame& frame, uint64_t timeout_us);
    
    // 便利方法：尝试通过DroneCAN发送
    bool try_send_via_dronecan(AP_HAL::CANFrame& frame, uint64_t timeout_us);
    
    // 便利方法：尝试通过HAL直接发送
    bool try_send_via_hal(AP_HAL::CANFrame& frame, uint64_t timeout_us);
    
    // 便利方法：尝试通过HAL CAN1发送
    bool try_send_via_hal_can1(AP_HAL::CANFrame& frame, uint64_t timeout_us);
    
    // CAN发送统计
    CANStats _can_stats;
}; 