#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// MIT motor command types
#define MIT_CMD_POSITION      0x01
#define MIT_CMD_SPEED        0x02
#define MIT_CMD_CURRENT      0x03
#define MIT_CMD_SET_ID       0x05

// KEGU motor command types
#define KEGU_CMD_INIT        0x01
#define KEGU_CMD_ENABLE_CUR  0x02
#define KEGU_CMD_SET_CUR     0x03

// Motor control modes
enum class MotorCtrlMode : uint8_t {
    POSITION = 0,
    SPEED = 1,
    CURRENT = 2,
    SET_ID = 3,
    SET_ZERO = 4,
    SET_MAX_SPD = 5,
    SET_MIN_SPD = 6,
    SET_INIT = 7,
    SET_SPD_ENABLE = 8,
    SET_CUR_ENABLE = 9
};

// Motor types
enum class MotorType : uint8_t {
    MIT = 0,
    KEGU = 1
};

// CAN channels
enum class CanChannel : uint8_t {
    CAN1 = 0,
    CAN2 = 1
};

// Motor status structures
struct Mit_Motor_Status {
    float velocity[8];    // RPM
    float position[8];    // Degrees
    float current[8];     // Amps
    float temperature[8]; // Celsius
    uint8_t state[8];    // Motor state flags
};

struct KeGu_Motor_Status {
    float speed[8];     // RPM
    float current[8];   // mA
    float position[8];  // pulse
};

// Main motor control functions
void mit_motor_send_command(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);
void kegu_motor_send_command(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);

// Helper functions
float get_motor_position(uint8_t can_id, uint8_t motor_id, MotorType type);
void process_mit_motor_frame(const AP_HAL::CANFrame &frame, Mit_Motor_Status &status);
void process_kegu_motor_frame(const AP_HAL::CANFrame &frame, KeGu_Motor_Status &status);

// CAN frame creation helpers
AP_HAL::CANFrame create_mit_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);
AP_HAL::CANFrame create_kegu_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);

// Singleton instance
class CAN_Robot_Tx_Process {
public:
    static CAN_Robot_Tx_Process *get_singleton() {
        return _singleton;
    }

    // Initialize the processor
    void init(void);
    
    // Process incoming motor commands
    void process_motor_command(uint8_t can_id, uint8_t motor_id, MotorType type, 
                             MotorCtrlMode mode, float value);

    // Get motor status
    const Mit_Motor_Status& get_mit_status() const { return _mit_status; }
    const KeGu_Motor_Status& get_kegu_status() const { return _kegu_status; }

private:
    static CAN_Robot_Tx_Process *_singleton;
    
    Mit_Motor_Status _mit_status;
    KeGu_Motor_Status _kegu_status;
    
    // Prevent direct construction
    CAN_Robot_Tx_Process() {}
};