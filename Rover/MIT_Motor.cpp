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
        {.can_id=2, .motor_id=1}
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

// Motor position getter function - requires implementation of CAN communication
float get_motor_position(uint8_t can_id, uint8_t motor_id)
{
    // This function should return the current motor position
    // Implementation depends on the actual CAN communication protocol
    // For now, return 0 as placeholder
    return 0.0f;
}

} // namespace MIT_Motor 