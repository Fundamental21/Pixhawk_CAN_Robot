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

// C++ namespace functions
#ifdef __cplusplus
namespace MIT_Motor {
    // 注意：核心函数声明现在统一在 CAN_Robot_Common.h 中定义
    // 这里只保留不在CAN_Robot_Common.h中的辅助函数
    
    // Utility functions
    float normalize_angle(float angle);
    float circular_diff(float from, float to);
    
    // Queue management functions
    void queue_push(PositionQueue* q, float pos);
    float queue_pop(PositionQueue* q);
    
    // Trapezoid planner functions
    void trapezoid_init(TrapezoidPlanner* planner, float initial_pos, float target_pos);
    void trapezoid_update(TrapezoidPlanner* planner, bool is_last_point);
    
    // Trajectory interpolation functions
    void init_trajectory_interpolation();
    bool add_key_point(const float joint_angles[6]);
    bool add_current_motor_positions();
    bool get_next_trajectory_point(float joint_angles[6]);
    bool generate_trajectory_interpolation(float Ts, float F, float ub_a, uint8_t max_points = 0);
    bool is_dense_queue_empty();
    bool is_sparse_queue_empty();
    uint8_t sparse_queue_count();
    
    // Direct transfer functions (for skipping interpolation)
    bool get_next_sparse_point(float joint_angles[6]);
    bool add_dense_point(const float joint_angles[6]);
    
    // 注意：夹爪控制函数现在统一在 CAN_Robot_Common.h 中声明
    
    // Convenient gripper control functions  
    void gripper_open(float speed_percentage);
    void gripper_close(float speed_percentage);
    void gripper_stop();
    
    // 双夹爪智能控制函数
    void process_dual_gripper_control();
    
    // 双夹爪控制函数（支持gripper_id参数）
    void set_gripper_current_by_id(uint8_t gripper_id, float current_value);
    void set_gripper_velocity_by_id(uint8_t gripper_id, float velocity_value);
}
#endif

#endif // __MIT_MOTOR_H 