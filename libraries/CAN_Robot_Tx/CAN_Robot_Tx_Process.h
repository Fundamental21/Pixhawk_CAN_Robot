#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "CAN_Robot_Common.h"

// Motor status structures with C++ style initialization
struct Mit_Motor_Status {
    float velocity[8]{};    // RPM
    float position[8]{};    // Degrees
    float current[8]{};     // Amps
    float temperature[8]{}; // Celsius
    uint8_t state[8]{};     // Motor state flags
};

struct KeGu_Motor_Status {
    float speed[8]{};     // RPM
    float current[8]{};   // mA
    float position[8]{};  // pulse
};

// CAN frame creation helpers (internal use only)
AP_HAL::CANFrame create_mit_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);
AP_HAL::CANFrame create_kegu_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);

// Frame processing functions (for CAN Rx modules)
void process_mit_motor_frame(const AP_HAL::CANFrame &frame, Mit_Motor_Status &status);
void process_kegu_motor_frame(const AP_HAL::CANFrame &frame, KeGu_Motor_Status &status);

// Motor position getter (for MIT_Motor module)
float get_motor_position(uint8_t can_id, uint8_t motor_id, MotorType type);

// Main CAN TX processor class (Singleton pattern)
class CAN_Robot_Tx_Process {
public:
    // Core interface functions - called by external modules
    static void init();
    static CAN_Robot_Tx_Process* get_singleton() { 
        return _singleton; 
    }

    // Main CAN frame creation function - called by AP_DroneCAN
    static AP_HAL::CANFrame create_motor_frame(uint8_t can_id, uint8_t motor_id, 
                                              MotorType motor_type, MotorControlMode mode, 
                                              float target_value);

    // Motor command processing - called by queue systems  
    void process_motor_command(uint8_t can_id, uint8_t motor_id, MotorType type, 
                             MotorControlMode mode, float value);

    // Status accessors (const getters)
    const Mit_Motor_Status& get_mit_status() const { 
        return _mit_status; 
    }
    
    const KeGu_Motor_Status& get_kegu_status() const { 
        return _kegu_status; 
    }

private:
    static CAN_Robot_Tx_Process *_singleton;
    
    Mit_Motor_Status _mit_status{};
    KeGu_Motor_Status _kegu_status{};
    
    // Private constructor for singleton pattern
    CAN_Robot_Tx_Process() = default;
    
    // Delete copy constructor and assignment operator
    CAN_Robot_Tx_Process(const CAN_Robot_Tx_Process&) = delete;
    CAN_Robot_Tx_Process& operator=(const CAN_Robot_Tx_Process&) = delete;
};