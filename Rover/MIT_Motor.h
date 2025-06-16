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

// Include CAN Robot common definitions and TX Queue for motor commands
#include <CAN_Robot_Tx/CAN_Robot_Common.h>
#include <CAN_Robot_Tx/CAN_Robot_Tx_Queue.h>

// All constants and structures now defined in CAN_Robot_Common.h

// Global motor instances array declaration
extern MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN];

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
    float get_motor_current(uint8_t can_id, uint8_t motor_id);
    float get_motor_velocity(uint8_t can_id, uint8_t motor_id);
    
    // Trajectory interpolation functions
    void init_trajectory_interpolation();
    bool add_key_point(const float joint_angles[6]);
    bool add_current_motor_positions();
    bool get_next_trajectory_point(float joint_angles[6]);
    bool generate_trajectory_interpolation(float Ts, float F, float ub_a, uint8_t max_points = 0);
    bool is_dense_queue_empty();
    bool is_sparse_queue_empty();
    uint8_t sparse_queue_count();
}
#endif

#endif // __MIT_MOTOR_H 