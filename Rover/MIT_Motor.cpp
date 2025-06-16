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

#include "MIT_Motor.h"
#include <CAN_Robot_interpolation/CAN_Robot_interpolation.h>

// Include CAN Robot Rx modules to access received motor status
#include <CAN_Robot_Rx/CAN_Robot_Rx_Queue.h>
#include <CAN_Robot_Rx/CAN_Robot_Rx_Process.h>

// ArduPilot includes (conditional, only if building within ArduPilot)
#ifdef ARDUPILOT_BUILD
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
extern const AP_HAL::HAL& hal;
#endif

//---------------------Global Variables---------------------
// Motor instances array definition with static initialization (preserving original structure)
MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN] = {
    // CAN1
    [0] = {
        {.can_id=1, .motor_id=1}, {.can_id=1, .motor_id=2}, {.can_id=1, .motor_id=3}, {.can_id=1, .motor_id=4},
        {.can_id=1, .motor_id=5}, {.can_id=1, .motor_id=6}, {.can_id=1, .motor_id=7}, {.can_id=1, .motor_id=8}
    },
    // CAN2
    [1] = {
        {.can_id=2, .motor_id=1}, {.can_id=2, .motor_id=2}, {.can_id=2, .motor_id=3}, {.can_id=2, .motor_id=4},
        {.can_id=2, .motor_id=5}, {.can_id=2, .motor_id=6}, {.can_id=2, .motor_id=7}, {.can_id=2, .motor_id=8}
    }
};

//---------------------Motor Instance Initialization---------------------
namespace MIT_Motor {

//------------------Utils Functions------------------
float normalize_angle(float angle) {
    angle = fmodf(angle, 360.0f);
    if(angle < -360.0f) angle += 720.0f;
    else if(angle > 360.0f) angle -= 720.0f;
    return angle;
}

float circular_diff(float from, float to) {
    float diff = to - from;
    diff = fmodf(diff + 540.0f, 360.0f) - 180.0f;
    return diff;
}
//------------------Utils Functions------------------

//---------------------Queue Management---------------------
void queue_push(PositionQueue* q, float pos) {
    if(!q) return;
    
    if(q->count < POSITION_QUEUE_SIZE) {
        q->data[q->tail] = normalize_angle(pos);
        q->tail = (q->tail + 1) % POSITION_QUEUE_SIZE;
        q->count++;
    }
}

float queue_pop(PositionQueue* q) {
    if(!q || q->count == 0) return NAN;
    
    float val = q->data[q->head];
    q->head = (q->head + 1) % POSITION_QUEUE_SIZE;
    q->count--;
    return val;
}
//---------------------Queue Management---------------------

//---------------------Trapezoid Planner---------------------
void trapezoid_init(TrapezoidPlanner* planner, float initial_pos, float target_pos) {
    if(!planner) return;
    
    planner->current_ref = normalize_angle(initial_pos);
    planner->target_pos = normalize_angle(target_pos);
    planner->current_vel = 0.0f;
    planner->max_velocity = MAX_VELOCITY;
    planner->max_accel = MAX_ACCEL;
    
    float diff = circular_diff(planner->current_ref, planner->target_pos);
    if(fabsf(diff) > 180.0f) {
        planner->target_pos += (diff > 0) ? -360.0f : 360.0f;
    }
    planner->is_terminated = false;
}

void trapezoid_update(TrapezoidPlanner* planner, bool is_last_point) {
    if(!planner || planner->is_terminated) return;

    float error = circular_diff(planner->current_ref, planner->target_pos);
    float error_abs = fabsf(error);
    float direction = error > 0 ? 1.0f : -1.0f;

    float decel_dist = (planner->current_vel * planner->current_vel) / (2 * planner->max_accel);
    
    if(error_abs <= decel_dist || is_last_point) {
        planner->current_vel -= direction * planner->max_accel * CONTROL_PERIOD;
        if((direction > 0 && planner->current_vel < 0) || 
           (direction < 0 && planner->current_vel > 0)) {
            planner->current_vel = 0;
        }
    } else {
        planner->current_vel += direction * planner->max_accel * CONTROL_PERIOD;
        if(fabsf(planner->current_vel) > planner->max_velocity) {
            planner->current_vel = direction * planner->max_velocity;
        }
    }
    
    planner->current_ref += planner->current_vel * CONTROL_PERIOD;
    planner->current_ref = normalize_angle(planner->current_ref);
    
    if(error_abs < 0.5f && fabsf(planner->current_vel) < 1.0f) {
        planner->is_terminated = true;
        planner->current_ref = planner->target_pos;
    }
}
//---------------------Trapezoid Planner---------------------

//---------------------Motor Control Handler---------------------
void MotorControl_Handler(MotorInstance* motor)
{
    if(!motor || !motor->enabled) return;

    // Get the queue singleton
    CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
    if (!queue) {
        return;
    }

    // Convert control mode to queue mode (both use same enum values)
    MotorControlMode queue_mode = motor->mode;

    // Queue the command
    queue->queue_motor_command(motor->can_id, motor->motor_id, 
                             motor->type == MOTOR_TYPE_MIT ? MOTOR_TYPE_MIT : MOTOR_TYPE_KEGU,
                             queue_mode, motor->target_value);
    
    // Request motor status
    if (motor->type == MOTOR_TYPE_MIT) {
        queue->queue_motor_command(motor->can_id, motor->motor_id, MOTOR_TYPE_MIT,
                                 CTRL_MODE_CURRENT, 0); // get current
        queue->queue_motor_command(motor->can_id, motor->motor_id, MOTOR_TYPE_MIT,
                                 CTRL_MODE_VELOCITY, 0); // get velocity
        queue->queue_motor_command(motor->can_id, motor->motor_id, MOTOR_TYPE_MIT,
                                 CTRL_MODE_POSITION, 0); // get position
    }
}
//---------------------Motor Control Handler---------------------

// Motor position getter function - gets MIT motor position from CAN1 Rx queue
float get_motor_position(uint8_t can_id, uint8_t motor_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // Only process CAN1 messages (channel 0 is CAN1 for motor)
    uint8_t can_channel = 0;  // CAN1
    
    // Get MIT motor status from CAN1
    const CAN_Robot_Rx_Process::MIT_Motor_Feedback& motor_feedback = 
        rx_processor->get_mit_motor_status(can_channel, motor_id);
    
    // Check if we have valid data
    if (motor_feedback.last_update_us > 0) {
        return motor_feedback.position;  // Return position in degrees
    }
    
    /* KEGU Motor interface (reserved for future use)
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_id);
    
    if (kegu_feedback.last_update_us > 0) {
        const float pulses_per_degree = 100.0f;
        return kegu_feedback.position / pulses_per_degree;
    }
    */
    
    return NAN;  // Return NAN to indicate no valid data
}

// Motor current getter function - gets MIT motor current from CAN1 Rx queue
float get_motor_current(uint8_t can_id, uint8_t motor_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // Only process CAN1 messages (channel 0 is CAN1 for motor)
    uint8_t can_channel = 0;  // CAN1
    
    // Get MIT motor status from CAN1
    const CAN_Robot_Rx_Process::MIT_Motor_Feedback& motor_feedback = 
        rx_processor->get_mit_motor_status(can_channel, motor_id);
    
    // Check if we have valid data
    if (motor_feedback.last_update_us > 0) {
        return motor_feedback.current;  // Return current in Amps
    }
    
    /* KEGU Motor interface (reserved for future use)
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_id);
    
    if (kegu_feedback.last_update_us > 0) {
        return kegu_feedback.current;
    }
    */
    
    return NAN;  // Return NAN to indicate no valid data
}

// Motor velocity getter function - gets MIT motor velocity from CAN1 Rx queue
float get_motor_velocity(uint8_t can_id, uint8_t motor_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // Only process CAN1 messages (channel 0 is CAN1 for motor)
    uint8_t can_channel = 0;  // CAN1
    
    // Get MIT motor status from CAN1
    const CAN_Robot_Rx_Process::MIT_Motor_Feedback& motor_feedback = 
        rx_processor->get_mit_motor_status(can_channel, motor_id);
    
    // Check if we have valid data
    if (motor_feedback.last_update_us > 0) {
        return motor_feedback.velocity;  // Return velocity in degrees/sec
    }
    
    /* KEGU Motor interface (reserved for future use)
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_id);
    
    if (kegu_feedback.last_update_us > 0) {
        return kegu_feedback.velocity;
    }
    */
    
    return NAN;  // Return NAN to indicate no valid data
}

//---------------------Trajectory Interpolation Functions---------------------
// 初始化轨迹插值系统
void init_trajectory_interpolation()
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (interpolator) {
        interpolator->init();
    }
}

// 添加关键点到稀疏队列
bool add_key_point(const float joint_angles[6])
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return false;
    }
    
    JointPoint point;
    for (uint8_t i = 0; i < 6; i++) {
        point.angles[i] = joint_angles[i];
    }
    
    return interpolator->sparse_queue_push(point);
}

// 获取当前电机位置并添加到稀疏队列
bool add_current_motor_positions()
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return false;
    }
    
    // 只有当稠密队列为空时才能获取新的电机位置
    if (!interpolator->dense_queue_is_empty()) {
        return false;
    }
    
    JointPoint current_point;
    // 获取6个关节电机的当前位置
    for (uint8_t i = 0; i < 6; i++) {
        // 假设关节电机在motor_instances[0][i]
        if (i < MOTORS_PER_CAN) {
            current_point.angles[i] = get_motor_position(motor_instances[0][i].can_id, 
                                                       motor_instances[0][i].motor_id);
        } else {
            current_point.angles[i] = 0.0f; // 默认值
        }
    }
    
    return interpolator->sparse_queue_push(current_point);
}

// 从稠密队列获取下一个轨迹点
bool get_next_trajectory_point(float joint_angles[6])
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return false;
    }
    
    JointPoint point;
    if (interpolator->dense_queue_pop(point)) {
        for (uint8_t i = 0; i < 6; i++) {
            joint_angles[i] = point.angles[i];
        }
        return true;
    }
    
    return false;
}

// 生成轨迹插值 - 5Hz调用，支持指定处理点数
bool generate_trajectory_interpolation(float Ts, float F, float ub_a, uint8_t max_points)
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return false;
    }
    
    // 只有当稀疏队列有数据且稠密队列为空时才生成新轨迹
    if (interpolator->sparse_queue_is_empty() || !interpolator->dense_queue_is_empty()) {
        return false;
    }
    
    return interpolator->generate_joint_trajectory(Ts, F, ub_a, max_points);
}

// 检查稠密队列状态
bool is_dense_queue_empty()
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return true;
    }
    
    return interpolator->dense_queue_is_empty();
}

// 检查稀疏队列状态
bool is_sparse_queue_empty()
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return true;
    }
    
    return interpolator->sparse_queue_is_empty();
}

// 获取稀疏队列中的点数
uint8_t sparse_queue_count()
{
    CAN_Robot_Interpolation* interpolator = CAN_Robot_Interpolation::get_singleton();
    if (!interpolator) {
        return 0;
    }
    
    return interpolator->sparse_queue_count();
}



} // namespace MIT_Motor 