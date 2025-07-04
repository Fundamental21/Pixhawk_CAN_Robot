#include <GCS_MAVLink/GCS.h>
#include "CAN_Robot_Tx_Queue.h"
#include <new>

// 单例实例 - 初始化为空指针，延迟初始化
CAN_Robot_Tx_Queue* CAN_Robot_Tx_Queue::_singleton = nullptr;

// 初始化静态实例 - 与Rx队列保持一致
void CAN_Robot_Tx_Queue::init(void)
{
    if (_singleton == nullptr) {
        _singleton = new (std::nothrow) CAN_Robot_Tx_Queue();
    }
}

CAN_Robot_Tx_Queue::CAN_Robot_Tx_Queue() :
    _queue_head(0),
    _queue_tail(0),
    _queue_count(0),
    _commands_sent(0),
    _commands_dropped(0),
    _last_log_ms(0)
{
    // Initialize all commands as unprocessed
    for (uint32_t i = 0; i < QUEUE_SIZE; i++) {
        _command_queue[i].processed = true; // Mark as processed initially
    }
}

void CAN_Robot_Tx_Queue::queue_motor_command(uint8_t can_id, uint8_t motor_id, MotorType motor_type,
                                            MotorControlMode mode, float target_value)
{
    WITH_SEMAPHORE(_queue_semaphore);
    
    if (_queue_full()) {
        _commands_dropped++;
        // Log dropped command for debugging
        AP::logger().Write_MessageF("CAN_TX_QUEUE: Command dropped - queue full. CAN:%d Motor:%d", 
                                   can_id, motor_id);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAN_TX_QUEUE: Command dropped - queue full");
        return;
    }
    
    // Add command to queue
    MotorCommand &cmd = _command_queue[_queue_tail];
    cmd.can_id = can_id;
    cmd.motor_id = motor_id;
    cmd.motor_type = motor_type;
    cmd.mode = mode;
    cmd.target_value = target_value;
    cmd.timestamp_us = AP_HAL::micros64();
    cmd.processed = false;
    
    _advance_tail();
    _queue_count++;
    
    // Debug logging for new commands
    AP::logger().Write_MessageF("CAN_TX_QUEUE: Queued CAN:%d Motor:%d Type:%d Mode:%d Value:%.2f", 
                               can_id, motor_id, (int)motor_type, (int)mode, (double)target_value);
    
    // Send command details to ground station
    const char* type_str = (motor_type == MotorType::MIT) ? "MIT" : "KEGU";
    const char* mode_str;
    switch (mode) {
        case MotorControlMode::POSITION:    mode_str = "POS"; break;
        case MotorControlMode::VELOCITY:    mode_str = "VEL"; break;
        case MotorControlMode::CURRENT:     mode_str = "CUR"; break;
        case MotorControlMode::INIT:        mode_str = "INIT"; break;
        case MotorControlMode::ENABLE_CUR:  mode_str = "EN_CUR"; break;
        case MotorControlMode::ENABLE_VEL:  mode_str = "EN_VEL"; break;
        default:                            mode_str = "UNK"; break;
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN_CMD: CAN%d M%d %s %s %.2f", 
                  can_id+1, motor_id, type_str, mode_str, (double)target_value);
}

bool CAN_Robot_Tx_Queue::get_next_command(MotorCommand &cmd)
{
    WITH_SEMAPHORE(_queue_semaphore);
    
    if (_queue_empty()) {
        return false;
    }
    
    // Get command from head of queue
    cmd = _command_queue[_queue_head];
    return true;
}

void CAN_Robot_Tx_Queue::mark_command_processed(void)
{
    WITH_SEMAPHORE(_queue_semaphore);
    
    if (!_queue_empty()) {
        _command_queue[_queue_head].processed = true;
        _advance_head();
        _queue_count--;
        _commands_sent++;
    }
}

uint32_t CAN_Robot_Tx_Queue::get_queue_size(void) const
{
    return _queue_count;
}

uint32_t CAN_Robot_Tx_Queue::get_commands_sent(void) const
{
    return _commands_sent;
}

uint32_t CAN_Robot_Tx_Queue::get_commands_dropped(void) const
{
    return _commands_dropped;
}

void CAN_Robot_Tx_Queue::log_status(void)
{
    uint32_t now_ms = AP_HAL::millis();
    
    // Log status every 5 seconds
    if (now_ms - _last_log_ms > 5000) {
        _last_log_ms = now_ms;
        
        AP::logger().Write_MessageF("CAN_TX_QUEUE: Size:%u Sent:%u Dropped:%u", 
                                   (unsigned)_queue_count, (unsigned)_commands_sent, 
                                   (unsigned)_commands_dropped);
        
        // Send to GCS if there are dropped commands
        if (_commands_dropped > 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN_TX_QUEUE: %u sent, %u dropped", 
                         (unsigned)_commands_sent, (unsigned)_commands_dropped);
        }
    }
}

bool CAN_Robot_Tx_Queue::_queue_full(void) const
{
    return _queue_count >= QUEUE_SIZE;
}

bool CAN_Robot_Tx_Queue::_queue_empty(void) const
{
    return _queue_count == 0;
}

void CAN_Robot_Tx_Queue::_advance_head(void)
{
    _queue_head = (_queue_head + 1) % QUEUE_SIZE;
}

void CAN_Robot_Tx_Queue::_advance_tail(void)
{
    _queue_tail = (_queue_tail + 1) % QUEUE_SIZE;
}

// Helper functions for motor control (called from MIT_Motor.cpp)
void handle_mit_motor(MotorInstance* motor)
{
    if (!motor || !motor->enabled) {
        return;
    }
    
    CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
    if (!queue) {
        return;
    }
    
    // Queue the motor command
    queue->queue_motor_command(motor->can_id, motor->motor_id, motor->type,
                              motor->mode, motor->target_value);
}

void handle_kegu_motor(MotorInstance* motor)
{
    if (!motor || !motor->enabled) {
        return;
    }
    
    CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
    if (!queue) {
        return;
    }
    
    // Queue the motor command
    queue->queue_motor_command(motor->can_id, motor->motor_id, motor->type,
                              motor->mode, motor->target_value);
} 