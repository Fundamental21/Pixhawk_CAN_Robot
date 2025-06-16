#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdbool.h>

// MIT motor command types
#define MIT_CMD_POSITION      0x01
#define MIT_CMD_SPEED        0x02
#define MIT_CMD_CURRENT      0x03
#define MIT_CMD_SET_ID       0x05

// KEGU motor command types
#define KEGU_CMD_INIT        0x01
#define KEGU_CMD_ENABLE_CUR  0x02
#define KEGU_CMD_SET_CUR     0x03
#define KEGU_CMD_ENABLE_POS  0x03
#define KEGU_CMD_CURRENT     0x04
#define KEGU_CMD_POSITION    0x05

// CAN channels
enum class CanChannel : uint8_t {
    CAN1 = 0,
    CAN2 = 1
};

// 常用常量
#define MAX_MOTORS_PER_CAN 8

// Motor control command constants - using specific prefixes to avoid conflicts
#define MIT_MOTOR_POSITION 1
#define MIT_MOTOR_SPEED 2
#define MIT_MOTOR_CURRENT 3
#define MIT_MOTOR_SET_ID 5

// Legacy aliases for backward compatibility - only when not in AP_DroneCAN context
// #ifndef AP_DRONECAN_INCLUDED
// #define POSITION MIT_MOTOR_POSITION
// #define SPEED MIT_MOTOR_SPEED
// #define CURRENT MIT_MOTOR_CURRENT
// #define SET_ID MIT_MOTOR_SET_ID
// #endif

// Motor type and control mode definitions
typedef enum {
    MOTOR_TYPE_MIT = 0,
    MOTOR_TYPE_KEGU = 1
} MotorType;

typedef enum {
    CTRL_MODE_POSITION = 0,
    CTRL_MODE_VELOCITY = 1, 
    CTRL_MODE_CURRENT = 2,
    CTRL_MODE_INIT = 3,
    CTRL_MODE_ENABLE_CUR = 4,
    CTRL_MODE_ENABLE_POS = 5,
    CTRL_MODE_MAX = 6
} MotorControlMode;

// 统一的常量定义
#define USART_RX_BUF_LENGHT     256     // 统一使用256
#define USART1_RX_BUF_LENGHT    9

#define MAX_CAN_NUM   2                 // 统一使用2个CAN通道
#define MOTORS_PER_CAN 8
#define POSITION_QUEUE_SIZE 100         // 统一使用100
#define POSITION_TOLERANCE  0.1f
#define MAX_VELOCITY     10.0f          // deg/s
#define MAX_ACCEL        20.0f          // deg/s^2
#define CONTROL_PERIOD   0.01f          // 10ms

// Joint and motor constants
#define JOINT_MOTOR_COUNT 6

// 统一的结构体定义
typedef struct {
    float angles[6];
} JointAngles;

typedef struct {
    float current_ref; 
    float current_vel; 
    float target_pos;
    float max_velocity;
    float max_accel;
    float accel_phase_time;
    float cruise_phase_time;
    float decel_phase_time;
    float total_time;
    float start_pos;
    float current_time;
    bool is_terminated;
} TrapezoidPlanner;

typedef struct {
    float data[POSITION_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} PositionQueue;

typedef struct {
    // id
    uint8_t can_id;
    uint8_t motor_id;
    
    // control state
    MotorControlMode mode;
    float target_value;
    float last_position;
    
    // motor state
    bool enabled;
    bool first_command;
    MotorType type;
	
    // position queue
    PositionQueue queue;
    
    // interpolation
    TrapezoidPlanner planner;
} MotorInstance;

// 额外的结构体定义（来自Rover.h）
typedef struct {
    float x, y, z;
    float roll, pitch, yaw;
} Pose;

typedef struct {
    JointAngles solutions[8];
    uint8_t count;
} IKSolutions;

typedef struct {
    float link_lengths[6];
    float joint_limits[6][2];  // [joint][min/max]
    char arm_side[2];
} RobotArmConfig; 