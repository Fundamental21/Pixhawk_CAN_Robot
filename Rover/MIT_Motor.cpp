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
// Motor instances array definition with proper initialization
MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN];

//---------------------Motor Instance Initialization---------------------
namespace MIT_Motor {

// Direct transfer queue for sparse to dense conversion (skipping interpolation)
static struct DirectTransferQueue {
    float points[50][6];  // 存储50个关键点，每个点6个关节角度
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    
    DirectTransferQueue() : head(0), tail(0), count(0) {}
    
    bool is_empty() const { return count == 0; }
    bool is_full() const { return count >= 50; }
    
    bool push(const float joint_angles[6]) {
        if (is_full()) return false;
        
        for (uint8_t i = 0; i < 6; i++) {
            points[tail][i] = joint_angles[i];
        }
        tail = (tail + 1) % 50;
        count++;
        return true;
    }
    
    bool pop(float joint_angles[6]) {
        if (is_empty()) return false;
        
        for (uint8_t i = 0; i < 6; i++) {
            joint_angles[i] = points[head][i];
        }
        head = (head + 1) % 50;
        count--;
        return true;
    }
} sparse_to_dense_queue;

// Initialize motor instances with proper values
void init_motor_instances() {
    // Initialize CAN1 motors
    for (uint8_t i = 0; i < MOTORS_PER_CAN; i++) {
        motor_instances[0][i].can_id = 0;           // CAN总线ID：0=CAN1
        motor_instances[0][i].motor_id = i + 1;     // 电机ID：1,2,3,4,5,6
        motor_instances[0][i].mode = MotorControlMode::POSITION;
        motor_instances[0][i].target_value = 0.0f;
        motor_instances[0][i].last_position = 0.0f;
        motor_instances[0][i].enabled = false;
        motor_instances[0][i].first_command = true;
        motor_instances[0][i].type = MotorType::MIT;
        // queue and planner are already initialized by their constructors
    }
    
    // Initialize CAN2 motors
    for (uint8_t i = 0; i < MOTORS_PER_CAN; i++) {
        motor_instances[1][i].can_id = 1;           // CAN总线ID：1=CAN2
        motor_instances[1][i].motor_id = i + 1;     // 电机ID：1,2,3,4,5,6
        motor_instances[1][i].mode = MotorControlMode::POSITION;
        motor_instances[1][i].target_value = 0.0f;
        motor_instances[1][i].last_position = 0.0f;
        motor_instances[1][i].enabled = false;
        motor_instances[1][i].first_command = true;
        motor_instances[1][i].type = MotorType::MIT;
        // queue and planner are already initialized by their constructors
    }
}

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
    CAN_Robot_Interpolation& interpolator = CAN_Robot_Interpolation::get_instance();
    interpolator.init();
}

// 添加关键点到稀疏队列
bool add_key_point(const float joint_angles[6])
{
    // 直接添加到直接传递队列，跳过插值系统
    return sparse_to_dense_queue.push(joint_angles);
}

// 获取当前电机位置并添加到稀疏队列
bool add_current_motor_positions()
{
    CAN_Robot_Interpolation& interpolator = CAN_Robot_Interpolation::get_instance();
    
    // 检查是否所有运动都已完成
    if (!interpolator.is_motion_complete()) {
        return false;
    }
    
    JointAngles current_angles;
    // 获取6个关节电机的当前位置
    for (uint8_t i = 0; i < 6; i++) {
        // 假设关节电机在motor_instances[0][i]
        if (i < MOTORS_PER_CAN) {
            current_angles.angles[i] = get_motor_position(motor_instances[0][i].can_id, 
                                                       motor_instances[0][i].motor_id);
        } else {
            current_angles.angles[i] = 0.0f; // 默认值
        }
    }
    
    interpolator.set_all_joints_target(current_angles);
    return true;
}

// 从稠密队列获取下一个轨迹点
bool get_next_trajectory_point(float joint_angles[6])
{
    // 直接从传递队列获取下一个轨迹点
    return sparse_to_dense_queue.pop(joint_angles);
}

// 生成轨迹插值 - 5Hz调用，支持指定处理点数
bool generate_trajectory_interpolation(float Ts, float F, float ub_a, uint8_t max_points)
{
    CAN_Robot_Interpolation& interpolator = CAN_Robot_Interpolation::get_instance();
    
    // 执行插值更新
    return interpolator.execute_interpolation();
}

// 检查稠密队列状态
bool is_dense_queue_empty()
{
    // 直接传递模式下，稠密队列就是我们的传递队列
    return sparse_to_dense_queue.is_empty();
}

// 检查稀疏队列状态
bool is_sparse_queue_empty()
{
    // 直接传递模式下，稀疏队列就是我们的传递队列
    return sparse_to_dense_queue.is_empty();
}

// 获取稀疏队列中的点数
uint8_t sparse_queue_count()
{
    // 返回直接传递队列中的点数
    return sparse_to_dense_queue.count;
}



// 从稀疏队列获取下一个点（直接传递用）
bool get_next_sparse_point(float joint_angles[6])
{
    // 从内部队列获取点
    return sparse_to_dense_queue.pop(joint_angles);
}

// 添加点到稠密队列（直接传递用）
bool add_dense_point(const float joint_angles[6])
{
    // 添加到内部队列
    return sparse_to_dense_queue.push(joint_angles);
}



} // namespace MIT_Motor 