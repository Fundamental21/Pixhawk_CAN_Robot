#include "CAN_Robot_Rx_Process.h"
#include <new>

// 单例实例
CAN_Robot_Rx_Process* CAN_Robot_Rx_Process::_singleton = nullptr;

// 初始化静态实例
void CAN_Robot_Rx_Process::init(void)
{
    if (_singleton == nullptr) {
        _singleton = new (std::nothrow) CAN_Robot_Rx_Process();
    }
}

CAN_Robot_Rx_Process::CAN_Robot_Rx_Process() :
    _mit_msg_count(0),
    _kegu_msg_count(0),
    _cmd_msg_count(0),
    _last_log_ms(0)
{
    // 初始化电机状态数组
    memset(_mit_motors, 0, sizeof(_mit_motors));
    memset(_kegu_motors, 0, sizeof(_kegu_motors));
}

void CAN_Robot_Rx_Process::process_all_rx_messages(void)
{
    CAN_Robot_Rx_Queue* rx_queue = CAN_Robot_Rx_Queue::get_singleton();
    if (!rx_queue) {
        return;
    }
    
    CAN_Robot_Rx_Queue::CANRxMessage msg;
    uint32_t processed_count = 0;
    
    while (rx_queue->get_next_message(msg)) {
        // 根据CAN ID类型分发消息
        if (is_mit_motor_id(msg.can_id)) {
            process_mit_motor_message(msg);
        } else if (is_kegu_motor_id(msg.can_id)) {
            process_kegu_motor_message(msg);
        } else {
            // 处理其他机器人通信消息
            process_robot_command_message(msg);
        }
        
        // 标记消息已处理
        rx_queue->mark_message_processed();
        processed_count++;
        
        // 防止单次处理过多消息影响实时性
        if (processed_count >= 20) {
            break;
        }
    }
    
    // 定期记录状态
    log_motor_status();
}

void CAN_Robot_Rx_Process::process_mit_motor_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    if (msg.can_channel > 1) return; // 只支持CAN1和CAN2
    
    uint8_t motor_id = extract_motor_id(msg.can_id);
    if (motor_id >= MAX_MOTORS_PER_CAN) return;
    
    MIT_Motor_Feedback &feedback = _mit_motors[msg.can_channel][motor_id];
    decode_mit_feedback(msg, feedback);
    feedback.last_update_us = msg.timestamp_us;
    
    _mit_msg_count++;
    
    // 记录详细日志
    AP::logger().Write_MessageF("MIT_RX: CAN%d ID:0x%X M%d Pos:%.2f Vel:%.2f Curr:%.2f Temp:%.1f", 
                               (int)msg.can_channel + 1, (unsigned)msg.can_id, motor_id,
                               feedback.position, feedback.velocity, 
                               feedback.current, feedback.temperature);
}

void CAN_Robot_Rx_Process::process_kegu_motor_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    if (msg.can_channel > 1) return; // 只支持CAN1和CAN2
    
    uint8_t motor_id = extract_motor_id(msg.can_id);
    if (motor_id >= MAX_MOTORS_PER_CAN) return;
    
    KEGU_Motor_Feedback &feedback = _kegu_motors[msg.can_channel][motor_id];
    decode_kegu_feedback(msg, feedback);
    feedback.last_update_us = msg.timestamp_us;
    
    _kegu_msg_count++;
    
    // 记录详细日志
    AP::logger().Write_MessageF("KEGU_RX: CAN%d ID:0x%X M%d Spd:%.1f Curr:%.1f Pos:%d", 
                               (int)msg.can_channel + 1, (unsigned)msg.can_id, motor_id,
                               feedback.speed, feedback.current, feedback.position);
}

void CAN_Robot_Rx_Process::process_robot_command_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    _cmd_msg_count++;
    
    // 处理机器人指令消息
    // 这里可以根据具体的机器人通信协议来实现
    AP::logger().Write_MessageF("ROBOT_CMD: CAN%d ID:0x%X DLC:%d [%02X %02X %02X %02X]", 
                               (int)msg.can_channel + 1, (unsigned)msg.can_id, msg.dlc,
                               (unsigned)msg.data[0], (unsigned)msg.data[1], 
                               (unsigned)msg.data[2], (unsigned)msg.data[3]);
}

bool CAN_Robot_Rx_Process::is_mit_motor_id(uint32_t can_id) const
{
    // MIT电机通常使用CAN ID 1-8
    return (can_id >= 1 && can_id <= 8);
}

bool CAN_Robot_Rx_Process::is_kegu_motor_id(uint32_t can_id) const
{
    // KEGU电机使用特定的ID模式
    uint16_t high_part = (can_id >> 8) & 0xFF;
    return (high_part == 0x02 || high_part == 0x03); // 反馈消息类型
}

uint8_t CAN_Robot_Rx_Process::extract_motor_id(uint32_t can_id) const
{
    // 对于MIT电机，ID就是CAN ID
    // 对于KEGU电机，ID在低字节
    return can_id & 0xFF;
}

void CAN_Robot_Rx_Process::decode_mit_feedback(const CAN_Robot_Rx_Queue::CANRxMessage &msg, MIT_Motor_Feedback &feedback)
{
    if (msg.dlc < 5) return; // 数据长度不足
    
    uint8_t cmd = msg.data[0];
    
    // 从数据字节1-4提取32位值（小端格式）
    int32_t raw_value = (msg.data[4] << 24) | (msg.data[3] << 16) | 
                        (msg.data[2] << 8) | msg.data[1];
    
    switch (cmd) {
        case 0x06: // 速度反馈
            feedback.velocity = raw_value * 0.01f; // 0.01 RPM 分辨率
            break;
        case 0x08: // 位置反馈
            feedback.position = raw_value * 0.01f; // 0.01 度分辨率
            break;
        case 0x04: // 电流反馈
            feedback.current = raw_value * 0.001f; // mA 分辨率转换为 A
            break;
        case 0x32: // 温度反馈
            feedback.temperature = static_cast<float>(raw_value);
            break;
        default:
            // 未知命令，记录错误
            feedback.error_code = cmd;
            break;
    }
}

void CAN_Robot_Rx_Process::decode_kegu_feedback(const CAN_Robot_Rx_Queue::CANRxMessage &msg, KEGU_Motor_Feedback &feedback)
{
    uint16_t msg_type = (msg.can_id >> 8) & 0xFF;
    
    if (msg_type == 0x02 && msg.dlc >= 6) {
        // 速度和电流反馈
        feedback.speed = *((int32_t*)msg.data);
        feedback.current = *((int16_t*)(msg.data + 4)) * 10.0f; // 缩放电流
    } else if (msg_type == 0x03 && msg.dlc >= 4) {
        // 位置反馈
        feedback.position = *((int32_t*)msg.data);
    }
    
    // 从DLC或数据中提取状态
    if (msg.dlc > 6) {
        feedback.status = msg.data[6];
    }
}

const CAN_Robot_Rx_Process::MIT_Motor_Feedback& CAN_Robot_Rx_Process::get_mit_motor_status(uint8_t can_channel, uint8_t motor_id) const
{
    static MIT_Motor_Feedback empty_feedback;
    
    if (can_channel > 1 || motor_id >= MAX_MOTORS_PER_CAN) {
        return empty_feedback;
    }
    
    return _mit_motors[can_channel][motor_id];
}

const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& CAN_Robot_Rx_Process::get_kegu_motor_status(uint8_t can_channel, uint8_t motor_id) const
{
    static KEGU_Motor_Feedback empty_feedback;
    
    if (can_channel > 1 || motor_id >= MAX_MOTORS_PER_CAN) {
        return empty_feedback;
    }
    
    return _kegu_motors[can_channel][motor_id];
}

void CAN_Robot_Rx_Process::log_motor_status(void)
{
    uint32_t now_ms = AP_HAL::millis();
    
    // 每10秒记录一次状态总结
    if (now_ms - _last_log_ms > 10000) {
        _last_log_ms = now_ms;
        
        AP::logger().Write_MessageF("CAN_ROBOT_STATUS: MIT:%u KEGU:%u CMD:%u", 
                                   (unsigned)_mit_msg_count, 
                                   (unsigned)_kegu_msg_count, 
                                   (unsigned)_cmd_msg_count);
        
        // 记录活跃电机状态
        for (uint8_t can = 0; can < 2; can++) {
            for (uint8_t motor = 0; motor < MAX_MOTORS_PER_CAN; motor++) {
                const MIT_Motor_Feedback &mit = _mit_motors[can][motor];
                if (mit.last_update_us > 0 && (AP_HAL::micros64() - mit.last_update_us) < 1000000) {
                    AP::logger().Write_MessageF("MIT_STATUS: CAN%d M%d Pos:%.2f Vel:%.2f", 
                                               can + 1, motor, mit.position, mit.velocity);
                }
                
                const KEGU_Motor_Feedback &kegu = _kegu_motors[can][motor];
                if (kegu.last_update_us > 0 && (AP_HAL::micros64() - kegu.last_update_us) < 1000000) {
                    AP::logger().Write_MessageF("KEGU_STATUS: CAN%d M%d Spd:%.1f Pos:%d", 
                                               can + 1, motor, kegu.speed, kegu.position);
                }
            }
        }
    }
} 