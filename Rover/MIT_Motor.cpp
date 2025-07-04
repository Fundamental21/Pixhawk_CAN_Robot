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
#include <CAN_Robot_Tx/CAN_Robot_Tx_Process.h>
#include <GCS_MAVLink/GCS.h>

// 外部全局变量声明（在Rover.cpp中定义的全局变量）
extern MotorInstance* gripper_motors[2];
extern bool grippers_initialized[2];
extern uint8_t kegu_external_commands[2];
extern bool kegu_external_commands_received[2];
extern uint32_t kegu_command_timestamps[2];
extern float gripper_positions[2];
extern float gripper_velocities[2];
extern float gripper_currents[2];

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
    // Initialize CAN1 motors - 修正注释和配置
    for (uint8_t i = 0; i < MOTORS_PER_CAN; i++) {
        motor_instances[0][i].can_id = 0;           // CAN总线ID：0=CAN1
        motor_instances[0][i].motor_id = i + 1;     // 电机ID：1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16
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
        motor_instances[1][i].motor_id = i + 1;     // 电机ID：1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16
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
    
    // Check if this is a KEGU motor (motor_id = 7 or 14 for grippers)
    if (motor_id == 7 || motor_id == 14) {
        // KEGU gripper motor mapping
        uint8_t motor_array_index;
        if (motor_id == 7) {
            motor_array_index = 6;   // ARM1夹爪: motor_id=7 -> 数组索引6
        } else {
            motor_array_index = 13;  // ARM2夹爪: motor_id=14 -> 数组索引13
        }
        
        const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
            rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
        
        if (kegu_feedback.last_update_us > 0) {
            const float pulses_per_degree = 100.0f;  // Conversion factor for KEGU motors
            return kegu_feedback.position / pulses_per_degree;  // Convert pulses to degrees
        }
    } else if ((motor_id >= 1 && motor_id <= 6) || (motor_id >= 8 && motor_id <= 13)) {
        // MIT motor handling - 修正索引映射
        uint8_t motor_array_index;
        if (motor_id >= 1 && motor_id <= 6) {
            // ARM1: motor_id 1-6 maps to array index 0-5
            motor_array_index = motor_id - 1;
        } else {
            // ARM2: motor_id 8-13 maps to array index 7-12
            motor_array_index = motor_id - 1;  // 8->7, 9->8, 10->9, 11->10, 12->11, 13->12
        }
        
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
    
    // Check if this is a KEGU motor (motor_id = 7 or 14 for grippers)
    if (motor_id == 7 || motor_id == 14) {
        // KEGU gripper motor mapping
        uint8_t motor_array_index;
        if (motor_id == 7) {
            motor_array_index = 6;   // ARM1夹爪: motor_id=7 -> 数组索引6
        } else {
            motor_array_index = 13;  // ARM2夹爪: motor_id=14 -> 数组索引13
        }
        
        const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
            rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
        
        if (kegu_feedback.last_update_us > 0) {
            return kegu_feedback.current;  // Return current in mA
        }
    } else if ((motor_id >= 1 && motor_id <= 6) || (motor_id >= 8 && motor_id <= 13)) {
        // MIT motor handling - 修正索引映射
        uint8_t motor_array_index;
        if (motor_id >= 1 && motor_id <= 6) {
            // ARM1: motor_id 1-6 maps to array index 0-5
            motor_array_index = motor_id - 1;
        } else {
            // ARM2: motor_id 8-13 maps to array index 7-12
            motor_array_index = motor_id - 1;  // 8->7, 9->8, 10->9, 11->10, 12->11, 13->12
        }
        
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
    
    // Check if this is a KEGU motor (motor_id = 7 or 14 for grippers)
    if (motor_id == 7 || motor_id == 14) {
        // KEGU gripper motor mapping
        uint8_t motor_array_index;
        if (motor_id == 7) {
            motor_array_index = 6;   // ARM1夹爪: motor_id=7 -> 数组索引6
        } else {
            motor_array_index = 13;  // ARM2夹爪: motor_id=14 -> 数组索引13
        }
        
        const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
            rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
        
        if (kegu_feedback.last_update_us > 0) {
            return kegu_feedback.speed;  // KEGU uses 'speed' field (RPM)
        }
    } else if ((motor_id >= 1 && motor_id <= 6) || (motor_id >= 8 && motor_id <= 13)) {
        // MIT motor handling - 修正索引映射
        uint8_t motor_array_index;
        if (motor_id >= 1 && motor_id <= 6) {
            // ARM1: motor_id 1-6 maps to array index 0-5
            motor_array_index = motor_id - 1;
        } else {
            // ARM2: motor_id 8-13 maps to array index 7-12
            motor_array_index = motor_id - 1;  // 8->7, 9->8, 10->9, 11->10, 12->11, 13->12
        }
        
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

//---------------------双夹爪支持函数---------------------
// 根据夹爪ID获取位置（支持双夹爪系统）
float get_gripper_position_by_id(uint8_t gripper_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    // 夹爪ID到数组索引的映射
    // gripper_id=0 -> motor_id=7 -> array_index=6 (ARM1夹爪)
    // gripper_id=1 -> motor_id=14 -> array_index=13 (ARM2夹爪) 
    uint8_t motor_array_index;
    
    if (gripper_id == 0) {
        motor_array_index = 6;  // ARM1夹爪: motor_id=7 -> 数组索引6（0-based）
    } else if (gripper_id == 1) {
        motor_array_index = 13; // ARM2夹爪: motor_id=14 -> 数组索引13（0-based）
    } else {
        return NAN;  // 无效的夹爪ID
    }
    
    uint8_t can_channel = 0;  // CAN1
    
    // Get KEGU motor status from CAN1
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
    
    if (kegu_feedback.last_update_us > 0) {
        const float pulses_per_degree = 100.0f;  // 根据实际电机规格调整
        return kegu_feedback.position / pulses_per_degree;
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

// 根据夹爪ID获取速度
float get_gripper_velocity_by_id(uint8_t gripper_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    uint8_t motor_array_index;
    
    if (gripper_id == 0) {
        motor_array_index = 6;  // ARM1夹爪，数组索引6
    } else if (gripper_id == 1) {
        motor_array_index = 13; // ARM2夹爪，数组索引13
    } else {
        return NAN;  // 无效的夹爪ID
    }
    
    uint8_t can_channel = 0;  // CAN1
    
    // Get KEGU motor status from CAN1
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
    
    if (kegu_feedback.last_update_us > 0) {
        return kegu_feedback.speed;  // KEGU速度直接使用
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

// 根据夹爪ID获取电流
float get_gripper_current_by_id(uint8_t gripper_id)
{
    // Get the CAN Rx processor singleton
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (!rx_processor) {
        return NAN;  // Return NAN to indicate no valid data
    }
    
    uint8_t motor_array_index;
    
    if (gripper_id == 0) {
        motor_array_index = 6;  // ARM1夹爪，数组索引6
    } else if (gripper_id == 1) {
        motor_array_index = 13; // ARM2夹爪，数组索引13
    } else {
        return NAN;  // 无效的夹爪ID
    }
    
    uint8_t can_channel = 0;  // CAN1
    
    // Get KEGU motor status from CAN1
    const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& kegu_feedback = 
        rx_processor->get_kegu_motor_status(can_channel, motor_array_index);
    
    if (kegu_feedback.last_update_us > 0) {
        return kegu_feedback.current;  // 直接返回mA单位
    }
    
    return NAN;  // Return NAN to indicate no valid data
}

//---------------------双夹爪智能控制函数---------------------
// 双夹爪智能控制函数 - 处理状态获取、速度控制、外部命令等
void process_dual_gripper_control()
{
    // 为每个夹爪进行控制处理
    for (uint8_t gripper_id = 0; gripper_id < 2; gripper_id++) {
        if (!grippers_initialized[gripper_id]) {
            continue; // 跳过未初始化的夹爪
        }
        
        MotorInstance* gripper_motor = gripper_motors[gripper_id];
        
        // 获取夹爪电机状态数据 - 夹爪电机会自动上报数据
        gripper_positions[gripper_id] = get_gripper_position_by_id(gripper_id);
        gripper_velocities[gripper_id] = get_gripper_velocity_by_id(gripper_id);
        gripper_currents[gripper_id] = get_gripper_current_by_id(gripper_id);
        
        // 使用日志记录方法打印KEGU电机电流 - 每1000ms打印一次
        static uint32_t last_current_print_ms[2] = {0, 0};
        uint32_t current_print_now_ms = AP_HAL::millis();
        if (current_print_now_ms - last_current_print_ms[gripper_id] >= 1000) {  // 每1000ms打印一次电流值
            // 使用AP::logger()方法记录电流到日志
            AP::logger().Write_MessageF("ARM%d_KEGU_CURRENT: %.2fmA", gripper_id+1, gripper_currents[gripper_id]);
            
            // 同时使用GCS发送文本消息
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ARM%d KEGU Current: %.2fmA", gripper_id+1, gripper_currents[gripper_id]);
            
            last_current_print_ms[gripper_id] = current_print_now_ms;
        }

        // KEGU电机速度控制 - 初始化完成后切换到速度控制模式
        static bool kegu_mode_switched[2] = {false, false};
        static bool first_velocity_set[2] = {false, false};
        static float current_target_velocity[2] = {0.0f, 0.0f};
        
        if (!kegu_mode_switched[gripper_id] && gripper_motor->mode == CTRL_MODE_INIT) {
            // 初始化完成，切换到速度控制模式
            gripper_motor->mode = CTRL_MODE_VELOCITY;
            kegu_mode_switched[gripper_id] = true;
            
            #ifdef ARDUPILOT_BUILD
            hal.console->printf("ARM%d KEGU: Switched to velocity control mode\n", gripper_id+1);
            #endif
            
            // 添加模式切换调试信息
            AP::logger().Write_MessageF("ARM%d_KEGU_MODE: Switched from INIT to VELOCITY control", gripper_id+1);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU Gripper: Ready for velocity control", gripper_id+1);
        }
        
        if (kegu_mode_switched[gripper_id]) {
            // 检查是否有外部命令正在处理（现在两个夹爪都支持外部命令）
            bool external_command_active = kegu_external_commands_received[gripper_id];
            
            if (!external_command_active) {
                // 只有在没有外部命令时才执行默认的自动控制逻辑
                // 第一次循环控制时设置速度为0
                if (!first_velocity_set[gripper_id]) {
                    current_target_velocity[gripper_id] = 0.0f;
                    first_velocity_set[gripper_id] = true;
                    
                    #ifdef ARDUPILOT_BUILD
                    hal.console->printf("ARM%d KEGU: First velocity set to %.1f\n", gripper_id+1, current_target_velocity[gripper_id]);
                    #endif
                    
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU: Initial velocity set to 0", gripper_id+1);
                }
                
                // 监控电流，当电流>200mA后设置速度为0
                if (fabsf(gripper_currents[gripper_id]) > 200.0f) {
                    current_target_velocity[gripper_id] = 0.0f;
                    gripper_motor->target_value = 0.0f;
                    set_gripper_velocity_by_id(gripper_id, 0.0f);  // 使用支持双夹爪的函数
                    
                    AP::logger().Write_MessageF("ARM%d_KEGU: Current %.1fmA > 200mA threshold, stopping motor", 
                                               gripper_id+1, gripper_currents[gripper_id]);
                    
                    #ifdef ARDUPILOT_BUILD
                    hal.console->printf("ARM%d KEGU: Current %.1fmA > 200mA threshold, stopping motor\n", 
                                       gripper_id+1, gripper_currents[gripper_id]);
                    #endif
                    
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ARM%d KEGU: High current detected, motor stopped", gripper_id+1);
                }
                
                // 设置目标速度
                gripper_motor->target_value = current_target_velocity[gripper_id];
                if (gripper_id == 0) {
                    set_gripper_velocity(current_target_velocity[gripper_id]); // 向后兼容ARM1
                } else {
                    set_gripper_velocity_by_id(gripper_id, current_target_velocity[gripper_id]); // ARM2使用新函数
                }
            }
            
            #ifdef ARDUPILOT_BUILD
            static uint32_t last_debug_ms[2] = {0, 0};
            uint32_t debug_now_ms = AP_HAL::millis();
            if (debug_now_ms - last_debug_ms[gripper_id] >= 2000) {  // 每2秒打印一次调试信息
                hal.console->printf("ARM%d KEGU: Velocity control - Target: %.1f, Current: %.1fmA, Velocity: %.1f, ExtCmd: %s\n", 
                                   gripper_id+1, current_target_velocity[gripper_id], gripper_currents[gripper_id], 
                                   gripper_velocities[gripper_id], external_command_active ? "ACTIVE" : "INACTIVE");
                
                // 定期记录夹爪状态到日志
                #if HAL_LOGGING_ENABLED
                AP::logger().Write_MessageF("ARM%d_KEGU_STATUS: Target:%.1f Current:%.1fmA Velocity:%.1f ExtCmd:%s", 
                                           gripper_id+1, current_target_velocity[gripper_id], gripper_currents[gripper_id], 
                                           gripper_velocities[gripper_id], external_command_active ? "ACTIVE" : "INACTIVE");
                #endif
                
                last_debug_ms[gripper_id] = debug_now_ms;
            }
            #endif
        }
        
        // 处理外部KEGU控制命令（现在支持两个夹爪）
        if (kegu_mode_switched[gripper_id]) {
            static uint8_t last_processed_commands[2] = {0xFF, 0xFF};  // 跟踪每个夹爪最后处理的命令
            static bool commands_in_progress[2] = {false, false};      // 跟踪每个夹爪命令执行状态
            
            if (kegu_external_commands_received[gripper_id]) {
                uint32_t current_time_ms = AP_HAL::millis();
                
                // 检查命令是否超时（5秒）
                if (current_time_ms - kegu_command_timestamps[gripper_id] > 5000) {
                    kegu_external_commands_received[gripper_id] = false;
                    commands_in_progress[gripper_id] = false;
                    last_processed_commands[gripper_id] = 0xFF;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ARM%d KEGU: External command timeout", gripper_id+1);
                } else {
                    // 检查是否是新命令
                    bool is_new_command = (kegu_external_commands[gripper_id] != last_processed_commands[gripper_id]);
                    if (is_new_command) {
                        commands_in_progress[gripper_id] = false;  // 重置状态以处理新命令
                        last_processed_commands[gripper_id] = kegu_external_commands[gripper_id];
                    }
                    
                    // 处理有效的外部命令
                    switch (kegu_external_commands[gripper_id]) {
                        case 0x00: {
                            // 停止命令
                            current_target_velocity[gripper_id] = 0.0f;
                            gripper_motor->target_value = 0.0f;
                            set_gripper_velocity_by_id(gripper_id, 0.0f);
                            
                            AP::logger().Write_MessageF("ARM%d_KEGU_EXT_CMD: STOP command executed", gripper_id+1);
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU: STOP command executed", gripper_id+1);
                            
                            kegu_external_commands_received[gripper_id] = false; // 命令处理完成
                            commands_in_progress[gripper_id] = false;
                            break;
                        }
                        
                        case 0x01: {
                            // 反转命令 - 设置负速度，包含电流检测
                            if (!commands_in_progress[gripper_id]) {
                                current_target_velocity[gripper_id] = -2000.0f; // 反向速度
                                gripper_motor->target_value = current_target_velocity[gripper_id];
                                set_gripper_velocity_by_id(gripper_id, current_target_velocity[gripper_id]);
                                commands_in_progress[gripper_id] = true;
                                
                                AP::logger().Write_MessageF("ARM%d_KEGU_EXT_CMD: REVERSE command started", gripper_id+1);
                                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU: REVERSE command started", gripper_id+1);
                            }
                            
                            // 监控电流，当电流>200mA后停止
                            if (fabsf(gripper_currents[gripper_id]) > 200.0f) {
                                current_target_velocity[gripper_id] = 0.0f;
                                gripper_motor->target_value = 0.0f;
                                set_gripper_velocity_by_id(gripper_id, 0.0f);
                                
                                AP::logger().Write_MessageF("ARM%d_KEGU_EXT_CMD: REVERSE stopped - current %.1fmA > 200mA", 
                                                           gripper_id+1, gripper_currents[gripper_id]);
                                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU: REVERSE stopped - high current", gripper_id+1);
                                
                                commands_in_progress[gripper_id] = false;
                                kegu_external_commands_received[gripper_id] = false; // 命令处理完成
                            }
                            break;
                        }
                        
                        case 0x11: {
                            // 正转命令 - 设置正速度，包含电流检测
                            if (!commands_in_progress[gripper_id]) {
                                current_target_velocity[gripper_id] = 2000.0f; // 正向速度
                                gripper_motor->target_value = current_target_velocity[gripper_id];
                                set_gripper_velocity_by_id(gripper_id, current_target_velocity[gripper_id]);
                                commands_in_progress[gripper_id] = true;
                                
                                AP::logger().Write_MessageF("ARM%d_KEGU_EXT_CMD: FORWARD command started", gripper_id+1);
                                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU: FORWARD command started", gripper_id+1);
                            }
                            
                            // 监控电流，当电流>200mA后停止
                            if (fabsf(gripper_currents[gripper_id]) > 200.0f) {
                                current_target_velocity[gripper_id] = 0.0f;
                                gripper_motor->target_value = 0.0f;
                                set_gripper_velocity_by_id(gripper_id, 0.0f);
                                
                                AP::logger().Write_MessageF("ARM%d_KEGU_EXT_CMD: FORWARD stopped - current %.1fmA > 200mA", 
                                                           gripper_id+1, gripper_currents[gripper_id]);
                                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU: FORWARD stopped - high current", gripper_id+1);
                                
                                commands_in_progress[gripper_id] = false;
                                kegu_external_commands_received[gripper_id] = false; // 命令处理完成
                            }
                            break;
                        }
                        
                        default: {
                            // 未知命令
                            AP::logger().Write_MessageF("ARM%d_KEGU_EXT_CMD: Unknown command 0x%02X", gripper_id+1, kegu_external_commands[gripper_id]);
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ARM%d KEGU: Unknown command 0x%02X", gripper_id+1, kegu_external_commands[gripper_id]);
                            kegu_external_commands_received[gripper_id] = false; // 命令处理完成
                            commands_in_progress[gripper_id] = false;
                            break;
                        }
                    }
                }
            }
        }
    }
}

//---------------------双夹爪控制函数---------------------
// 根据夹爪ID设置电流控制 (支持双夹爪系统)
void set_gripper_current_by_id(uint8_t gripper_id, float current_value)
{
    if (gripper_id >= 2) {
        return;  // 无效的夹爪ID
    }
    
    MotorInstance* gripper_motor = gripper_motors[gripper_id];
    
    if (!gripper_motor->enabled) {
        return;
    }
    
    // 限制电流值在合理范围内 (mA单位)
    if (current_value > 500.0f) current_value = 500.0f;    // 最大500mA (0.5A)
    if (current_value < -500.0f) current_value = -500.0f;  // 最大500mA (0.5A)
    
    // KEGU电机需要先使能电流控制模式
    static bool current_mode_enabled[2] = {false, false};
    if (!current_mode_enabled[gripper_id]) {
        gripper_motor->mode = CTRL_MODE_ENABLE_CUR;
        gripper_motor->target_value = 0.0f;
        MotorControl_Handler(gripper_motor);
        current_mode_enabled[gripper_id] = true;
        
        #ifdef ARDUPILOT_BUILD
        hal.scheduler->delay_microseconds(1000);  // 短暂延迟确保使能成功
        #endif
    }
    
    gripper_motor->mode = CTRL_MODE_CURRENT;
    gripper_motor->target_value = current_value;
    
    // 使用现有的电机控制处理函数
    MotorControl_Handler(gripper_motor);
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("ARM%d Gripper Current Set: %.1f mA\n", gripper_id+1, current_value);
#endif
}

// 根据夹爪ID设置速度控制 (支持双夹爪系统)
void set_gripper_velocity_by_id(uint8_t gripper_id, float velocity_value)
{
    if (gripper_id >= 2) {
        return;  // 无效的夹爪ID
    }
    
    MotorInstance* gripper_motor = gripper_motors[gripper_id];
    
    if (!gripper_motor->enabled) {
        return;
    }
    
    // 限制速度值在合理范围内
    if (velocity_value > 3000.0f) velocity_value = 3000.0f;
    if (velocity_value < -3000.0f) velocity_value = -3000.0f;
    
    // KEGU电机需要先使能速度控制模式
    static bool velocity_mode_enabled[2] = {false, false};
    if (!velocity_mode_enabled[gripper_id]) {
        gripper_motor->mode = CTRL_MODE_ENABLE_VEL;
        gripper_motor->target_value = 0.0f;
        MotorControl_Handler(gripper_motor);
        velocity_mode_enabled[gripper_id] = true;
        
        #ifdef ARDUPILOT_BUILD
        hal.scheduler->delay_microseconds(1000);  // 短暂延迟确保使能成功
        #endif
    }
    
    gripper_motor->mode = CTRL_MODE_VELOCITY;
    gripper_motor->target_value = velocity_value;
    
    // 使用现有的电机控制处理函数
    MotorControl_Handler(gripper_motor);
    
#ifdef ARDUPILOT_BUILD
    hal.console->printf("ARM%d Gripper Velocity Set: %.2f deg/s\n", gripper_id+1, velocity_value);
#endif
}

} // namespace MIT_Motor
