#include "CAN_Robot_Tx_Process.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <cmath>

// Singleton instance
CAN_Robot_Tx_Process *CAN_Robot_Tx_Process::_singleton = nullptr;

// Initialize static instance
void CAN_Robot_Tx_Process::init(void)
{
    if (_singleton == nullptr)
    {
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

    if (motor_type == MotorType::MIT)
    {
        switch (mode)
        {
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
    }
    else if (motor_type == MotorType::KEGU)
    {
        switch (mode)
        {
        case MotorControlMode::INIT:
            frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_INIT, target_value);
            break;
        case MotorControlMode::ENABLE_CUR:
            frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_ENABLE_CUR, target_value);
            break;
        case MotorControlMode::ENABLE_VEL:
            frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_ENABLE_VEL, target_value);
            break;
        case MotorControlMode::CURRENT:
            frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_SET_CUR, target_value);
            break;
        case MotorControlMode::VELOCITY:
            frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_SET_VEL, target_value);
            break;
        default:
            // Return empty frame for unsupported modes
            frame = AP_HAL::CANFrame{}; // Use value initialization instead of memset
            break;
        }
    }
    else
    {
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

    switch (type)
    {
    case MotorType::MIT:
        switch (mode)
        {
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
        switch (mode)
        {
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
    frame.dlc = 5;

    // Convert float value to int32 based on command type
    int32_t int_value{0};
    uint8_t address{0};
    switch (cmd)
    {
    case MIT_CMD_POSITION:
        address = 0x1E;
        int_value = (int32_t)roundf(value / 360.0f * (262144.0f));
        break;
    case MIT_CMD_SPEED:
        address = 0x1D;
        int_value = (int32_t)roundf(value * 101.0f * (5.0f / 3.0f));
        break;
    case MIT_CMD_CURRENT:
        address = 0x1C;
        int_value = (int32_t)roundf(value * 1000.0f);
        break;
    }

    frame.data[0] = address;
    frame.data[1] = (uint8_t)(int_value >> 0);
    frame.data[2] = (uint8_t)(int_value >> 8);
    frame.data[3] = (uint8_t)(int_value >> 16);
    frame.data[4] = (uint8_t)(int_value >> 24);

    return frame;
}

// Create KEGU motor CAN frame - 根据KEGU协议实现
AP_HAL::CANFrame create_kegu_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame{};
    frame.dlc = 8;

    // 清零数据数组
    memset(frame.data, 0, 8);

    switch (cmd)
    {
    case KEGU_CMD_INIT:
        // 总线启动指令: CAN ID = 0x000, data = [0x01, 电机ID, 0x00, ...]
        frame.id = 0x000;
        frame.data[0] = 0x01;
        frame.data[1] = motor_id;
        // 其余字节已经被memset清零
        break;

    case KEGU_CMD_ENABLE_CUR:
        // 电机使能电流模式: CAN ID = 0x200 + 电机ID
        frame.id = 0x200 + motor_id;
        frame.data[0] = 0x00;
        frame.data[1] = 0x10; // 10 = 电流模式
        frame.data[2] = 0x0f;
        frame.data[3] = 0x00;
        // 最大速度设置为3000RPM (0x00000BB8)
        frame.data[4] = 0xB8;
        frame.data[5] = 0x0B;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        break;

    case KEGU_CMD_ENABLE_VEL:
        // 电机使能速度模式: CAN ID = 0x200 + 电机ID
        frame.id = 0x200 + motor_id;
        frame.data[0] = 0x00;
        frame.data[1] = 0x03; // 03 = 速度模式
        frame.data[2] = 0x0f;
        frame.data[3] = 0x00;
        // 最大速度设置为3000RPM (0x00000BB8)
        frame.data[4] = 0xB8;
        frame.data[5] = 0x0B;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        break;

    case KEGU_CMD_SET_CUR:
        // 设置电流: CAN ID = 0x400 + 电机ID
        frame.id = 0x400 + motor_id;
        // Byte0-3: 速度值(设为0，因为是电流控制)
        frame.data[0] = 0x00;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        // Byte4-5: 电流值(单位10mA)
        {
            int16_t current_10ma = static_cast<int16_t>(value / 10.0f); // mA转换为10mA单位
            frame.data[4] = static_cast<uint8_t>(current_10ma & 0xFF);
            frame.data[5] = static_cast<uint8_t>((current_10ma >> 8) & 0xFF);
        }
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        break;

    case KEGU_CMD_SET_VEL: // 速度控制
        // 设置速度: CAN ID = 0x400 + 电机ID
        frame.id = 0x400 + motor_id;
        // Byte0-3: 速度值(RPM)
        {
            int32_t speed_rpm = static_cast<int32_t>(value);
            frame.data[0] = static_cast<uint8_t>(speed_rpm & 0xFF);
            frame.data[1] = static_cast<uint8_t>((speed_rpm >> 8) & 0xFF);
            frame.data[2] = static_cast<uint8_t>((speed_rpm >> 16) & 0xFF);
            frame.data[3] = static_cast<uint8_t>((speed_rpm >> 24) & 0xFF);
        }
        // Byte4-7: 电流模式时用，速度模式填0
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        break;

    default:
        // 未知命令，返回空帧
        frame = AP_HAL::CANFrame{};
        break;
    }

    return frame;
}

// Process incoming MIT motor frame - 可能被CAN接收模块调用
void process_mit_motor_frame(const AP_HAL::CANFrame &frame, Mit_Motor_Status &status)
{
    const uint8_t motor_id = frame.id & 0x0F;
    if (motor_id >= 8)
    {
        return;
    }

    const uint8_t cmd = frame.data[0];
    const int32_t value = (static_cast<int32_t>(frame.data[1]) << 24) |
                          (static_cast<int32_t>(frame.data[2]) << 16) |
                          (static_cast<int32_t>(frame.data[3]) << 8) |
                          static_cast<int32_t>(frame.data[4]);

    switch (cmd)
    {
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
    if (motor_id >= 8)
    {
        return;
    }

    const uint8_t cmd = frame.data[0];
    const int32_t value = (static_cast<int32_t>(frame.data[1]) << 24) |
                          (static_cast<int32_t>(frame.data[2]) << 16) |
                          (static_cast<int32_t>(frame.data[3]) << 8) |
                          static_cast<int32_t>(frame.data[4]);

    switch (cmd)
    {
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
    CAN_Robot_Tx_Process *processor = CAN_Robot_Tx_Process::get_singleton();
    if (processor == nullptr)
    {
        return 0.0f;
    }

    if (type == MotorType::MIT)
    {
        const Mit_Motor_Status &status = processor->get_mit_status();
        if (motor_id < 8)
        {
            return status.position[motor_id];
        }
    }
    else if (type == MotorType::KEGU)
    {
        const KeGu_Motor_Status &status = processor->get_kegu_status();
        if (motor_id < 8)
        {
            return status.position[motor_id];
        }
    }

    return 0.0f;
}
