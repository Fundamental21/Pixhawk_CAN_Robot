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

// Include CAN Robot Rx modules to access received motor status
#include <CAN_Robot_Rx/CAN_Robot_Rx_Queue.h>
#include <CAN_Robot_Rx/CAN_Robot_Rx_Process.h>

// ArduPilot includes (conditional, only if building within ArduPilot)
#ifdef ARDUPILOT_BUILD
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
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

    // Queue the command - only send position control command
    queue->queue_motor_command(motor->can_id, motor->motor_id, 
                             motor->type == MOTOR_TYPE_MIT ? MOTOR_TYPE_MIT : MOTOR_TYPE_KEGU,
                             queue_mode, motor->target_value);
    
    // GET requests are handled by robot_can_tx_loop in AP_DroneCAN.cpp
    // Removed duplicate GET requests to avoid CAN bus congestion
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
    
    // Check if this is a KEGU motor (motor_id = 7 is KEGU gripper)
    if (motor_id == 7) {
        // KEGU gripper motor uses array index 6 (0-based)
        uint8_t motor_array_index = 6;
        const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
            rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
        
        if (kegu_feedback.last_update_us > 0) {
            const float pulses_per_degree = 100.0f;  // Conversion factor for KEGU motors
            return kegu_feedback.position / pulses_per_degree;  // Convert pulses to degrees
        }
    } else if (motor_id >= 1 && motor_id <= 6) {
        // MIT motor handling (motor_id 1-6 maps to array index 0-5)
        uint8_t motor_array_index = motor_id - 1;
        const CAN_Robot_Rx_Process::MIT_Motor_Feedback& motor_feedback = 
            rx_processor->get_mit_motor_status(can_channel, motor_array_index);
        
        // Check if we have valid data
        if (motor_feedback.last_update_us > 0) {
            return motor_feedback.position;  // Return position in degrees
        }
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

// Motor current getter function - gets motor current from CAN1 Rx queue (supports both MIT and KEGU)
float get_motor_current(uint8_t can_id, uint8_t motor_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // Only process CAN1 messages (channel 0 is CAN1 for motor)
    uint8_t can_channel = 0;  // CAN1
    
    // Check if this is a KEGU motor (motor_id = 7 is KEGU gripper)
    if (motor_id == 7) {
        // KEGU gripper motor uses array index 6 (0-based)
        uint8_t motor_array_index = 6;
        const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
            rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
        
        if (kegu_feedback.last_update_us > 0) {
            return kegu_feedback.current;  // Return current in mA
        }
    } else if (motor_id >= 1 && motor_id <= 6) {
        // MIT motor handling (motor_id 1-6 maps to array index 0-5)
        uint8_t motor_array_index = motor_id - 1;
        const CAN_Robot_Rx_Process::MIT_Motor_Feedback& motor_feedback = 
            rx_processor->get_mit_motor_status(can_channel, motor_array_index);
        
        // Check if we have valid data
        if (motor_feedback.last_update_us > 0) {
            return motor_feedback.current;  // Return current in mA
        }
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

// Motor velocity getter function - gets motor velocity from CAN1 Rx queue (supports both MIT and KEGU)
float get_motor_velocity(uint8_t can_id, uint8_t motor_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // Only process CAN1 messages (channel 0 is CAN1 for motor)
    uint8_t can_channel = 0;  // CAN1
    
    // Check if this is a KEGU motor (motor_id = 7 is KEGU gripper)
    if (motor_id == 7) {
        // KEGU gripper motor uses array index 6 (0-based)
        uint8_t motor_array_index = 6;
        const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
            rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
        
        if (kegu_feedback.last_update_us > 0) {
            return kegu_feedback.speed;  // KEGU uses 'speed' field (RPM)
        }
    } else if (motor_id >= 1 && motor_id <= 6) {
        // MIT motor handling (motor_id 1-6 maps to array index 0-5)
        uint8_t motor_array_index = motor_id - 1;
        const CAN_Robot_Rx_Process::MIT_Motor_Feedback& motor_feedback = 
            rx_processor->get_mit_motor_status(can_channel, motor_array_index);
        
        // Check if we have valid data
        if (motor_feedback.last_update_us > 0) {
            return motor_feedback.velocity;  // Return velocity in degrees/sec
        }
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

//---------------------Gripper Motor Control Functions---------------------
// Set gripper motor current control value (positive = close, negative = open)
// current_value: 输入毫安(mA)
void set_gripper_current(float current_value)
{
    // Access global motor_instances array (defined outside namespace)
    MotorInstance* gripper_motor = &::motor_instances[0][6];  // Motor ID 7
    
    if (!gripper_motor->enabled) {
        return;
    }
    
    // 限制电流值在合理范围内 (mA单位)
    if (current_value > 500.0f) current_value = 500.0f;    // 最大500mA (0.5A)
    if (current_value < -500.0f) current_value = -500.0f;  // 最大500mA (0.5A)
    
    // KEGU电机需要先使能电流控制模式
    if (gripper_motor->first_command) {
        gripper_motor->mode = CTRL_MODE_ENABLE_CUR;
        gripper_motor->target_value = 0.0f;
        MotorControl_Handler(gripper_motor);
        gripper_motor->first_command = false;
        
        #ifdef ARDUPILOT_BUILD
        hal.scheduler->delay_microseconds(1000);  // 短暂延迟确保使能成功
        #endif
    }
    
    gripper_motor->mode = CTRL_MODE_CURRENT;
    gripper_motor->target_value = current_value;
    
    // 使用现有的电机控制处理函数
    MotorControl_Handler(gripper_motor);
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("Gripper Current Set: %.1f mA\n", current_value);
#endif
}

// Set gripper motor velocity control value (positive = close, negative = open)
void set_gripper_velocity(float velocity_value)
{
    // Access global motor_instances array (defined outside namespace)
    MotorInstance* gripper_motor = &::motor_instances[0][6];  // Motor ID 7
    
    if (!gripper_motor->enabled) {
        return;
    }
    
    // 限制速度值在合理范围内
    if (velocity_value > 3000.0f) velocity_value = 3000.0f;
    if (velocity_value < -3000.0f) velocity_value = -3000.0f;
    
    // KEGU电机需要先使能速度控制模式
    static bool velocity_mode_enabled = false;
    if (!velocity_mode_enabled) {
        gripper_motor->mode = CTRL_MODE_ENABLE_VEL;
        gripper_motor->target_value = 0.0f;
        MotorControl_Handler(gripper_motor);
        velocity_mode_enabled = true;
        
        #ifdef ARDUPILOT_BUILD
        hal.scheduler->delay_microseconds(1000);  // 短暂延迟确保使能成功
        #endif
    }
    
    gripper_motor->mode = CTRL_MODE_VELOCITY;
    gripper_motor->target_value = velocity_value;
    
    // 使用现有的电机控制处理函数
    MotorControl_Handler(gripper_motor);
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("Gripper Velocity Set: %.2f deg/s\n", velocity_value);
#endif
}

// Get gripper motor status - KEGU电机从接收处理器获取数据
float get_gripper_position()
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // KEGU夹爪电机使用CAN1 (channel 0), Motor ID 7 -> 数组索引6
    uint8_t can_channel = 0;  // CAN1
    uint8_t motor_array_index = 6;  // 电机ID=7对应数组索引6（0-based）
    
    // Get KEGU motor status from CAN1
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
    
    if (kegu_feedback.last_update_us > 0) {
        // KEGU电机位置可能以脉冲为单位，需要转换
        const float pulses_per_degree = 100.0f;  // 根据实际电机规格调整
        return kegu_feedback.position / pulses_per_degree;
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

float get_gripper_velocity() 
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // KEGU夹爪电机使用CAN1 (channel 0), Motor ID 7 -> 数组索引6
    uint8_t can_channel = 0;  // CAN1
    uint8_t motor_array_index = 6;  // 电机ID=7对应数组索引6（0-based）
    
    // Get KEGU motor status from CAN1
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
    
    if (kegu_feedback.last_update_us > 0) {
        return kegu_feedback.speed;  // KEGU速度直接使用
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

float get_gripper_current()
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return -2;  // Return -2 to indicate no valid rx_processor
    }
    
    // KEGU夹爪电机使用CAN1 (channel 0), Motor ID 7 -> 数组索引6
    uint8_t can_channel = 0;  // CAN1
    uint8_t motor_array_index = 6;  // 电机ID=7对应数组索引6（0-based）
    
    // Get KEGU motor status from CAN1
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
    
    // 添加详细的调试信息
    #ifdef ARDUPILOT_BUILD
    static uint32_t last_debug_time = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    // 每秒打印一次调试信息
    if (now_ms - last_debug_time > 1000) {
        last_debug_time = now_ms;
        
        // 检查last_update_us是否有效
        AP::logger().Write_MessageF("GRIPPER_CURRENT_DEBUG: current=%.1f last_update=%llu now=%llu age_ms=%u", 
                                   kegu_feedback.current, 
                                   (unsigned long long)kegu_feedback.last_update_us,
                                   (unsigned long long)AP_HAL::micros64(),
                                   (unsigned)((AP_HAL::micros64() - kegu_feedback.last_update_us) / 1000));
        
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Gripper Current Debug: %.1fmA update_us=%llu age_ms=%u", 
                     kegu_feedback.current, (unsigned long long)kegu_feedback.last_update_us,
                     (unsigned)((AP_HAL::micros64() - kegu_feedback.last_update_us) / 1000));
    }
    #endif
    
    return kegu_feedback.current;  // 直接返回mA单位
}

} // namespace MIT_Motor
