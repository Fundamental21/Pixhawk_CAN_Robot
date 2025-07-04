#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Logger/AP_Logger.h>
#include "CAN_Robot_Common.h"

class CAN_Robot_Tx_Queue {
public:
    // CLASS_NO_COPY macro remains
    CLASS_NO_COPY(CAN_Robot_Tx_Queue);

    // Motor command structure for queuing
    struct MotorCommand {
        uint8_t can_id;
        uint8_t motor_id;
        MotorType motor_type;
        MotorControlMode mode;
        float target_value;
        uint64_t timestamp_us;
        bool processed;
        
        MotorCommand() : can_id(0), motor_id(0), motor_type(MotorType::MIT),
                        mode(MotorControlMode::POSITION), target_value(0.0f), 
                        timestamp_us(0), processed(false) {}
    };

    // 单例模式接口 - 与Rx队列保持一致
    static void init(void);
    static CAN_Robot_Tx_Queue *get_singleton(void) { return _singleton; }

    // Main interface functions
    void queue_motor_command(uint8_t can_id, uint8_t motor_id, MotorType motor_type,
                           MotorControlMode mode, float target_value);
    bool get_next_command(MotorCommand &cmd);
    void mark_command_processed(void);
    
    // Status and logging
    uint32_t get_queue_size(void) const;
    uint32_t get_commands_sent(void) const;
    uint32_t get_commands_dropped(void) const;
    void log_status(void);

private:
    static CAN_Robot_Tx_Queue *_singleton;
    static const uint32_t QUEUE_SIZE = 200;  // 扩展到200以适配双臂高负载
    
    // 私有构造函数 - 防止外部直接创建实例
    CAN_Robot_Tx_Queue();
    
    // Command queue
    MotorCommand _command_queue[QUEUE_SIZE];
    uint32_t _queue_head;
    uint32_t _queue_tail;
    uint32_t _queue_count;
    
    // Statistics
    uint32_t _commands_sent;
    uint32_t _commands_dropped;
    uint32_t _last_log_ms;
    
    // Thread safety
    HAL_Semaphore _queue_semaphore;
    
    // Internal helper functions
    bool _queue_full(void) const;
    bool _queue_empty(void) const;
    void _advance_head(void);
    void _advance_tail(void);
};

// Helper functions for motor control (to be called from MIT_Motor.cpp)
// 注意：helper函数现在统一在 CAN_Robot_Common.h 中声明

// Motor instance structure now defined in MIT_Motor.h 