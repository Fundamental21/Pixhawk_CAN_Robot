#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

// MIT Motor control command types
#define MIT_CMD_POSITION    1
#define MIT_CMD_SPEED       2
#define MIT_CMD_CURRENT     3
#define MIT_CMD_SET_ID      5

// Kegu Motor control modes
#define KEGU_CMD_INIT           0x01
#define KEGU_CMD_ENABLE_CUR     0x02
#define KEGU_CMD_ENABLE_POS     0x03
#define KEGU_CMD_CURRENT        0x04
#define KEGU_CMD_POSITION       0x05

// Motor types - using enum class to match CAN_Robot_Tx_Process.h
enum class MotorType : uint8_t {
    MIT = 0,
    KEGU = 1
};

// Control modes - using enum class to match CAN_Robot_Tx_Process.h
enum class MotorControlMode : uint8_t {
    CTRL_MODE_POSITION = 0,
    CTRL_MODE_VELOCITY = 1,
    CTRL_MODE_CURRENT = 2,
    CTRL_MODE_INIT = 3,
    CTRL_MODE_ENABLE_CUR = 4,
    CTRL_MODE_ENABLE_POS = 5,
    CTRL_MODE_MAX = 6
};

// Compatibility aliases for existing code
#define MOTOR_TYPE_MIT MotorType::MIT
#define MOTOR_TYPE_KEGU MotorType::KEGU
#define CTRL_MODE_POSITION MotorControlMode::CTRL_MODE_POSITION
#define CTRL_MODE_VELOCITY MotorControlMode::CTRL_MODE_VELOCITY
#define CTRL_MODE_CURRENT MotorControlMode::CTRL_MODE_CURRENT
#define CTRL_MODE_INIT MotorControlMode::CTRL_MODE_INIT
#define CTRL_MODE_ENABLE_CUR MotorControlMode::CTRL_MODE_ENABLE_CUR
#define CTRL_MODE_ENABLE_POS MotorControlMode::CTRL_MODE_ENABLE_POS

class CAN_Robot_Tx_Queue {
public:
    CAN_Robot_Tx_Queue();
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
        
        MotorCommand() : can_id(0), motor_id(0), motor_type(MOTOR_TYPE_MIT),
                        mode(CTRL_MODE_POSITION), target_value(0.0f), 
                        timestamp_us(0), processed(false) {}
    };

    static CAN_Robot_Tx_Queue *get_singleton(void) { return _singleton; }
    static CAN_Robot_Tx_Queue *_singleton;

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
    static const uint32_t QUEUE_SIZE = 100;
    
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
void handle_mit_motor(struct MotorInstance* motor);
void handle_kegu_motor(struct MotorInstance* motor);

// Motor instance structure (forward declaration from MIT_Motor.h)
struct MotorInstance {
    uint8_t can_id;
    uint8_t motor_id;
    MotorControlMode mode;
    float target_value;
    bool enabled;
    MotorType type;
    bool first_command;
    
    // Additional fields that might be in MIT_Motor.h
    void* queue;       // PositionQueue*
    void* planner;     // TrapezoidPlanner*
}; 