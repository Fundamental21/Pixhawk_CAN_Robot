#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdbool.h>

// MIT motor command types
#define MIT_CMD_POSITION     0x1E
#define MIT_CMD_SPEED        0x1D
#define MIT_CMD_CURRENT      0x1C
#define MIT_CMD_SET_ID       0x1B

// KEGU motor command types - 根据实际协议更新
#define KEGU_CMD_INIT        0x01  // 总线启动指令
#define KEGU_CMD_ENABLE_CUR  0x02  // 使能电流模式
#define KEGU_CMD_ENABLE_VEL  0x03  // 使能速度模式 
#define KEGU_CMD_SET_CUR     0x04  // 设置电流值
#define KEGU_CMD_SET_VEL     0x05  // 设置速度值
// 兼容别名
#define KEGU_CMD_ENABLE_POS  0x03  // 兼容旧名称
#define KEGU_CMD_CURRENT     0x04  // 兼容别名
#define KEGU_CMD_POSITION    0x05  // 兼容别名

// CAN channels
enum class CanChannel : uint8_t {
    CAN1 = 0,
    CAN2 = 1
};

// 常用常量 - 扩展以支持双臂配置
// ARM1: motor_id 1-6 (数组索引0-5)  
// ARM2: motor_id 8-13 (数组索引7-12)
// ARM2夹爪: motor_id 14 (数组索引13)
#define MAX_MOTORS_PER_CAN 16

// Motor control command constants - using specific prefixes to avoid conflicts
#define MIT_MOTOR_POSITION 1
#define MIT_MOTOR_SPEED 2
#define MIT_MOTOR_CURRENT 3
#define MIT_MOTOR_SET_ID 5

// Additional MIT motor commands
#define SET_MAX_SPD 6
#define SET_MIN_SPD 7
#define SET_ZERO 8

// Legacy aliases for backward compatibility - only when not in AP_DroneCAN context
// #ifndef AP_DRONECAN_INCLUDED
// #define POSITION MIT_MOTOR_POSITION
// #define SPEED MIT_MOTOR_SPEED
// #define CURRENT MIT_MOTOR_CURRENT
// #define SET_ID MIT_MOTOR_SET_ID
// #endif

// Motor type and control mode definitions - C++ style enums
enum class MotorType : uint8_t {
    MIT = 0,
    KEGU = 1
};

enum class MotorControlMode : uint8_t {
    POSITION = 0,
    VELOCITY = 1, 
    CURRENT = 2,
    INIT = 3,
    ENABLE_CUR = 4,
    ENABLE_VEL = 5,
    MAX = 6
};

// Legacy definitions for backward compatibility
#define MOTOR_TYPE_MIT     MotorType::MIT
#define MOTOR_TYPE_KEGU    MotorType::KEGU
#define CTRL_MODE_POSITION MotorControlMode::POSITION
#define CTRL_MODE_VELOCITY MotorControlMode::VELOCITY
#define CTRL_MODE_CURRENT  MotorControlMode::CURRENT
#define CTRL_MODE_INIT     MotorControlMode::INIT
#define CTRL_MODE_ENABLE_CUR MotorControlMode::ENABLE_CUR
#define CTRL_MODE_ENABLE_VEL MotorControlMode::ENABLE_VEL
#define CTRL_MODE_ENABLE_POS MotorControlMode::ENABLE_POS  // 兼容别名

// Legacy MotorCtrlMode for backward compatibility
using MotorCtrlMode = MotorControlMode;

// 统一的常量定义
constexpr uint16_t USART_RX_BUF_LENGHT = 256;     // 统一使用256
constexpr uint8_t USART1_RX_BUF_LENGHT = 9;

constexpr uint8_t MAX_CAN_NUM = 2;                 // 统一使用2个CAN通道
constexpr uint8_t MOTORS_PER_CAN = 16;              // 扩展以支持双臂配置
constexpr uint16_t POSITION_QUEUE_SIZE = 100;     // 统一使用100
constexpr float POSITION_TOLERANCE = 0.1f;
constexpr float MAX_VELOCITY = 10.0f;             // deg/s
constexpr float MAX_ACCEL = 20.0f;                // deg/s^2
constexpr float CONTROL_PERIOD = 0.01f;           // 10ms

// Joint and motor constants
constexpr uint8_t JOINT_MOTOR_COUNT = 6;

// 统一的结构体定义
struct JointAngles {
    float angles[6];
    
    JointAngles() {
        for (int i = 0; i < 6; i++) {
            angles[i] = 0.0f;
        }
    }
};

struct TrapezoidPlanner {
    float current_ref{0.0f}; 
    float current_vel{0.0f}; 
    float target_pos{0.0f};
    float max_velocity{MAX_VELOCITY};
    float max_accel{MAX_ACCEL};
    float accel_phase_time{0.0f};
    float cruise_phase_time{0.0f};
    float decel_phase_time{0.0f};
    float total_time{0.0f};
    float start_pos{0.0f};
    float current_time{0.0f};
    bool is_terminated{false};
};

struct PositionQueue {
    float data[POSITION_QUEUE_SIZE];
    uint16_t head{0};
    uint16_t tail{0};
    uint16_t count{0};
    
    PositionQueue() {
        for (uint16_t i = 0; i < POSITION_QUEUE_SIZE; i++) {
            data[i] = 0.0f;
        }
    }
};

struct MotorInstance {
    // ID information
    uint8_t can_id{0};
    uint8_t motor_id{0};
    
    // Control state
    MotorControlMode mode{MotorControlMode::POSITION};
    float target_value{0.0f};
    float last_position{0.0f};
    
    // Motor state
    bool enabled{false};
    bool first_command{true};
    MotorType type{MotorType::MIT};
	
    // Position queue
    PositionQueue queue;
    
    // Interpolation planner
    TrapezoidPlanner planner;
};

// 额外的结构体定义（来自Rover.h）
struct Pose {
    float x{0.0f}, y{0.0f}, z{0.0f};
    float roll{0.0f}, pitch{0.0f}, yaw{0.0f};
};

struct IKSolutions {
    JointAngles solutions[8];
    uint8_t count{0};
};

struct RobotArmConfig {
    float link_lengths[6];
    float joint_limits[6][2];  // [joint][min/max]
    char arm_side[2];
    
    RobotArmConfig() {
        for (int i = 0; i < 6; i++) {
            link_lengths[i] = 0.0f;
            joint_limits[i][0] = 0.0f;
            joint_limits[i][1] = 0.0f;
        }
        arm_side[0] = 'L';
        arm_side[1] = '\0';
    }
};

// ===========================================
// 统一的函数声明 - 解决所有冲突
// ===========================================

// 前向声明
struct MotorInstance;

// MIT Motor namespace functions (declared here to resolve conflicts)
namespace MIT_Motor {
    // 电机实例初始化
    void init_motor_instances();
    
    // 电机控制处理
    void MotorControl_Handler(MotorInstance* motor);
    
    // 电机状态获取 - 统一接口
    float get_motor_position(uint8_t can_id, uint8_t motor_id);
    float get_motor_velocity(uint8_t can_id, uint8_t motor_id);
    float get_motor_current(uint8_t can_id, uint8_t motor_id);
    
    // 夹爪控制函数 - 单夹爪接口(向后兼容)
    void set_gripper_current(float current_value);
    void set_gripper_velocity(float velocity_value);
    float get_gripper_position();
    float get_gripper_velocity();
    float get_gripper_current();
    
    // 夹爪控制函数 - 双夹爪接口
    float get_gripper_position_by_id(uint8_t gripper_id);
    float get_gripper_velocity_by_id(uint8_t gripper_id);
    float get_gripper_current_by_id(uint8_t gripper_id);
}

// CAN_Robot_Tx_Process functions
namespace CAN_Robot_Tx_Process_Functions {
    // 注意：这个版本有额外的MotorType参数，与MIT_Motor版本不同
    float get_motor_position_with_type(uint8_t can_id, uint8_t motor_id, MotorType type);
}

// CAN_Robot_Tx_Process 全局函数声明 (在CAN_Robot_Tx_Process.cpp中实现)
float get_motor_position_with_type(uint8_t can_id, uint8_t motor_id, MotorType type);

// CAN_Robot_Tx_Queue functions  
void handle_mit_motor(MotorInstance* motor);
void handle_kegu_motor(MotorInstance* motor); 