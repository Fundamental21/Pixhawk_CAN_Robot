#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger.h>
#include "../CAN_Robot_Tx/CAN_Robot_Common.h"
#include "CAN_Robot_Rx_Queue.h"

// 专门处理机器人CAN消息的处理器
class CAN_Robot_Rx_Process {
public:
    static void init(void);
    static CAN_Robot_Rx_Process* get_singleton(void) { return _singleton; }
    
    // 主要处理函数 - 从队列处理所有消息
    void process_all_rx_messages(void);
    
    // 具体消息处理函数
    void process_mit_motor_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg);
    void process_kegu_motor_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg);
    void process_robot_command_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg);
    
    // MIT电机状态结构
    struct MIT_Motor_Feedback {
        float position;     // 位置 (度)
        float velocity;     // 速度 (RPM)
        float current;      // 电流 (A)
        float temperature;  // 温度 (°C)
        uint8_t error_code; // 错误代码
        uint64_t last_update_us; // 最后更新时间
        
        MIT_Motor_Feedback() {
            position = 0.0f;
            velocity = 0.0f;
            current = 0.0f;
            temperature = 0.0f;
            error_code = 0;
            last_update_us = 0;
        }
    };
    
    // KEGU电机状态结构
    struct KEGU_Motor_Feedback {
        float speed;        // 速度 (RPM)
        float current;      // 电流 (mA)
        int32_t position;   // 位置 (脉冲)
        uint8_t status;     // 状态标志
        uint64_t last_update_us; // 最后更新时间
        
        KEGU_Motor_Feedback() {
            speed = 0.0f;
            current = 0.0f;
            position = 0;
            status = 0;
            last_update_us = 0;
        }
    };
    
    // 获取电机状态
    const MIT_Motor_Feedback& get_mit_motor_status(uint8_t can_channel, uint8_t motor_id) const;
    const KEGU_Motor_Feedback& get_kegu_motor_status(uint8_t can_channel, uint8_t motor_id) const;
    
    // 统计信息
    uint32_t get_mit_message_count(void) const { return _mit_msg_count; }
    uint32_t get_kegu_message_count(void) const { return _kegu_msg_count; }
    uint32_t get_command_message_count(void) const { return _cmd_msg_count; }
    
    // 日志记录
    void log_motor_status(void);
    
    // 辅助函数（设为public以便外部调用）
    bool is_mit_motor_id(uint32_t motor_id) const;
    bool is_kegu_motor_id(uint32_t motor_id) const;
    uint8_t extract_motor_id(uint32_t motor_id) const;

private:
    static CAN_Robot_Rx_Process *_singleton;
    
    // 电机状态数组 [CAN通道][电机ID]
    MIT_Motor_Feedback _mit_motors[2][MAX_MOTORS_PER_CAN];   // CAN1, CAN2
    KEGU_Motor_Feedback _kegu_motors[2][MAX_MOTORS_PER_CAN]; // CAN1, CAN2
    
    // 统计信息
    uint32_t _mit_msg_count;
    uint32_t _kegu_msg_count;
    uint32_t _cmd_msg_count;
    uint32_t _last_log_ms;
    
    void decode_mit_feedback(const CAN_Robot_Rx_Queue::CANRxMessage &msg, MIT_Motor_Feedback &feedback);
    void decode_kegu_feedback(const CAN_Robot_Rx_Queue::CANRxMessage &msg, KEGU_Motor_Feedback &feedback);
    
    // 防止直接构造
    CAN_Robot_Rx_Process();
}; 