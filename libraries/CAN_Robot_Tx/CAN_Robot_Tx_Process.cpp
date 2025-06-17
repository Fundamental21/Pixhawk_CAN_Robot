#include "CAN_Robot_Tx_Process.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <cmath>

// Singleton instance
CAN_Robot_Tx_Process* CAN_Robot_Tx_Process::_singleton = nullptr;

// Initialize static instance
void CAN_Robot_Tx_Process::init(void)
{
    if (_singleton == nullptr) {
        _singleton = new CAN_Robot_Tx_Process();
    }
    
    // Use C++ style initialization instead of memset for non-trivial types
    _singleton->_mit_status = Mit_Motor_Status{};
    _singleton->_kegu_status = KeGu_Motor_Status{};
}

// Create CAN frame from motor command (unified method) - 主要被外部调用的函数
AP_HAL::CANFrame CAN_Robot_Tx_Process::create_motor_frame(uint8_t can_id, uint8_t motor_id, 
                                                           MotorType motor_type, MotorControlMode mode, 
                                                           float target_value)
{
    AP_HAL::CANFrame frame{};
    
    if (motor_type == MotorType::MIT) {
        switch (mode) {
            case MotorControlMode::POSITION:
                frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_POSITION, target_value);
                break;
            case MotorControlMode::VELOCITY:
                frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_SPEED, target_value);
                break;
            case MotorControlMode::CURRENT:
                frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_CURRENT, target_value);
                break;
            default:
                // Return empty frame for unsupported modes
                frame = AP_HAL::CANFrame{}; // Use value initialization instead of memset
                break;
        }
    } else if (motor_type == MotorType::KEGU) {
        switch (mode) {
            case MotorControlMode::INIT:
                frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_INIT, target_value);
                break;
            case MotorControlMode::ENABLE_CUR:
                frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_ENABLE_CUR, target_value);
                break;
            case MotorControlMode::CURRENT:
                frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_SET_CUR, target_value);
                break;
            default:
                // Return empty frame for unsupported modes
                frame = AP_HAL::CANFrame{}; // Use value initialization instead of memset
                break;
        }
    } else {
        // Return empty frame for unsupported motor types
        frame = AP_HAL::CANFrame{}; // Use value initialization instead of memset
    }
    
    return frame;
}

// Process motor commands and create CAN frames
void CAN_Robot_Tx_Process::process_motor_command(uint8_t can_id, uint8_t motor_id, 
                                               MotorType type, MotorControlMode mode, 
                                               float value)
{
    AP_HAL::CANFrame frame{};
    
    switch (type) {
        case MotorType::MIT:
            switch (mode) {
                case MotorControlMode::POSITION:
                    frame = create_motor_frame(can_id, motor_id, type, mode, value);
                    break;
                case MotorControlMode::VELOCITY:
                    frame = create_motor_frame(can_id, motor_id, type, mode, value);
                    break;
                case MotorControlMode::CURRENT:
                    frame = create_motor_frame(can_id, motor_id, type, mode, value);
                    break;
                default:
                    return;
            }
            break;
            
        case MotorType::KEGU:
            switch (mode) {
                case MotorControlMode::INIT:
                    frame = create_motor_frame(can_id, motor_id, type, mode, value);
                    break;
                case MotorControlMode::ENABLE_CUR:
                    frame = create_motor_frame(can_id, motor_id, type, mode, value);
                    break;
                case MotorControlMode::CURRENT:
                    frame = create_motor_frame(can_id, motor_id, type, mode, value);
                    break;
                default:
                    return;
            }
            break;
    }
    
    // Log the CAN frame creation
    AP::logger().Write_MessageF("CAN_TX: ID=0x%X, Type=%u, Mode=%u, Value=%.2f",
                               static_cast<unsigned>(can_id), 
                               static_cast<unsigned>(type), 
                               static_cast<unsigned>(mode), 
                               static_cast<double>(value));
}

// Create MIT motor CAN frame - 内部辅助函数
AP_HAL::CANFrame create_mit_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame{};
    frame.id = motor_id;
    frame.dlc = 8;
    
    // Convert float value to int32 based on command type
    int32_t int_value{0};
    switch (cmd) {
        case MIT_CMD_POSITION:
            int_value = static_cast<int32_t>(value * 100.0f); // 0.01 degree resolution
            break;
        case MIT_CMD_SPEED:
            int_value = static_cast<int32_t>(value * 100.0f); // 0.01 RPM resolution
            break;
        case MIT_CMD_CURRENT:
            int_value = static_cast<int32_t>(value * 1000.0f); // mA resolution
            break;
        default:
            int_value = static_cast<int32_t>(value);
            break;
    }
    
    frame.data[0] = cmd;
    frame.data[1] = 0;
    frame.data[2] = static_cast<uint8_t>((int_value >> 24) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((int_value >> 16) & 0xFF);
    frame.data[4] = static_cast<uint8_t>((int_value >> 8) & 0xFF);
    frame.data[5] = static_cast<uint8_t>(int_value & 0xFF);
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    return frame;
}

// Create KEGU motor CAN frame - 内部辅助函数
AP_HAL::CANFrame create_kegu_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame{};
    frame.id = motor_id;
    frame.dlc = 8;
    
    const int32_t int_value = static_cast<int32_t>(value);
    
    frame.data[0] = cmd;
    frame.data[1] = static_cast<uint8_t>((int_value >> 24) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((int_value >> 16) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((int_value >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(int_value & 0xFF);
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    return frame;
}

// Process incoming MIT motor frame - 可能被CAN接收模块调用
void process_mit_motor_frame(const AP_HAL::CANFrame &frame, Mit_Motor_Status &status)
{
    const uint8_t motor_id = frame.id & 0x0F;
    if (motor_id >= 8) {
        return;
    }
    
    const uint8_t cmd = frame.data[0];
    const int32_t value = (static_cast<int32_t>(frame.data[1]) << 24) | 
                         (static_cast<int32_t>(frame.data[2]) << 16) | 
                         (static_cast<int32_t>(frame.data[3]) << 8) | 
                          static_cast<int32_t>(frame.data[4]);
                    
    switch (cmd) {
        case 0x06: // Velocity
            status.velocity[motor_id] = static_cast<float>(value) * 0.01f;
            break;
        case 0x08: // Position
            status.position[motor_id] = static_cast<float>(value) * 0.01f;
            break;
        case 0x04: // Current
            status.current[motor_id] = static_cast<float>(value) * 0.001f;
            break;
        case 0x32: // Temperature
            status.temperature[motor_id] = static_cast<float>(value);
            break;
        default:
            break;
    }
}

// Process incoming KEGU motor frame - 可能被CAN接收模块调用
void process_kegu_motor_frame(const AP_HAL::CANFrame &frame, KeGu_Motor_Status &status)
{
    const uint8_t motor_id = frame.id & 0x0F;
    if (motor_id >= 8) {
        return;
    }
    
    const uint8_t cmd = frame.data[0];
    const int32_t value = (static_cast<int32_t>(frame.data[1]) << 24) | 
                         (static_cast<int32_t>(frame.data[2]) << 16) | 
                         (static_cast<int32_t>(frame.data[3]) << 8) | 
                          static_cast<int32_t>(frame.data[4]);
                    
    switch (cmd) {
        case 0x01: // Speed
            status.speed[motor_id] = static_cast<float>(value);
            break;
        case 0x02: // Current
            status.current[motor_id] = static_cast<float>(value) * 0.001f;
            break;
        case 0x03: // Position
            status.position[motor_id] = static_cast<float>(value);
            break;
        default:
            break;
    }
}

// Get motor position - 可能被MIT_Motor模块调用
float get_motor_position(uint8_t can_id, uint8_t motor_id, MotorType type)
{
    CAN_Robot_Tx_Process* processor = CAN_Robot_Tx_Process::get_singleton();
    if (processor == nullptr) {
        return 0.0f;
    }
    
    if (type == MotorType::MIT) {
        const Mit_Motor_Status& status = processor->get_mit_status();
        if (motor_id < 8) {
            return status.position[motor_id];
        }
    } else if (type == MotorType::KEGU) {
        const KeGu_Motor_Status& status = processor->get_kegu_status();
        if (motor_id < 8) {
            return status.position[motor_id];
        }
    }
    
    return 0.0f;
}

