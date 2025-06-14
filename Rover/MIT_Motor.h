/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __MIT_MOTOR_H
#define __MIT_MOTOR_H

#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>
#include <cmath>

// Include CAN Robot TX Queue for motor commands

#include <CAN_Robot_Tx/CAN_Robot_Tx_Queue.h>

// Constants from MIT_MotorV0.h
#define USART_RX_BUF_LENGHT     128
#define USART1_RX_BUF_LENGHT    9

#define MAX_CAN_NUM   1     // CAN number 
#define MOTORS_PER_CAN 8
#define POSITION_QUEUE_SIZE 50
#define POSITION_TOLERANCE  0.1f
#define MAX_VELOCITY     10.0f    // deg/s
#define MAX_ACCEL        20.0f   // deg/s^2
#define CONTROL_PERIOD   0.01f    // 10ms

// Motor control constants
#define POSITION 1
#define SPEED 2
#define CURRENT 3
#define SET_ID 5

// Joint angles structure
typedef struct {
    float angles[6];
} JointAngles;

// Structures from MIT_MotorV0.h with C++ compatibility
typedef struct {
    float current_ref; 
    float current_vel; 
    float target_pos;
    float max_velocity;
    float max_accel;
    bool  is_terminated;
} TrapezoidPlanner;

typedef struct {
    float data[POSITION_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} PositionQueue;

typedef enum {
    CTRL_MODE_POSITION,
    CTRL_MODE_VELOCITY, 
    CTRL_MODE_CURRENT,
    CTRL_MODE_INIT,
    CTRL_MODE_ENABLE_CUR,
    CTRL_MODE_ENABLE_POS,
    CTRL_MODE_MAX
} MotorControlMode;

typedef enum {
    MOTOR_TYPE_MIT = 0,
    MOTOR_TYPE_KEGU = 1
} MotorType;

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
    MotorType type;  // Add motor type field
	
    // position queue
    PositionQueue queue;
    
    // interpolation
    TrapezoidPlanner planner;
} MotorInstance;

// Global motor instances array declaration
extern MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN];

static const uint8_t JOINT_MOTOR_COUNT = 6;
MotorInstance* joint_motor_list[JOINT_MOTOR_COUNT] = {
    &motor_instances[0][0],
    &motor_instances[0][1],
    &motor_instances[0][2],
    &motor_instances[0][3],
    &motor_instances[0][4],
    &motor_instances[0][5]
};

// Function declarations from MIT_MotorV0.h
extern void mit_motor_task (void const* argu);

// External functions for Tihu motor control
#ifdef __cplusplus
extern "C" {
#endif
void Tihu_motor_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value);
void Tihu_motor_one_byte_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t cmd);
#ifdef __cplusplus
}
#endif

// C++ namespace functions
#ifdef __cplusplus
namespace MIT_Motor {
    // Utility functions
    float normalize_angle(float angle);
    float circular_diff(float from, float to);
    
    // Queue management functions
    void queue_push(PositionQueue* q, float pos);
    float queue_pop(PositionQueue* q);
    
    // Trapezoid planner functions
    void trapezoid_init(TrapezoidPlanner* planner, float initial_pos, float target_pos);
    void trapezoid_update(TrapezoidPlanner* planner, bool is_last_point);
    
    // Motor control functions
    void MotorControl_Handler(MotorInstance* motor);
    float get_motor_position(uint8_t can_id, uint8_t motor_id);
}
#endif

#endif // __MIT_MOTOR_H 