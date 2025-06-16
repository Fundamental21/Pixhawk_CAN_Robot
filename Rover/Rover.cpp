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

/*
   This is the ArduRover firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Randy Mackay, Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See https://ardupilot.org/dev for details
*/

#include "Rover.h"

// Include MIT Motor for robot arm control
#include "MIT_Motor.h"

// Include CAN Robot modules
#include <CAN_Robot_Rx/CAN_Robot_Rx_Queue.h>
#include <CAN_Robot_Rx/CAN_Robot_Rx_Process.h>
#include <CAN_Robot_Tx/CAN_Robot_Tx_Queue.h>
#include <CAN_Robot_Tx/CAN_Robot_Tx_Process.h>
#include <CAN_Robot_interpolation/CAN_Robot_interpolation.h>

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include "AP_Gripper/AP_Gripper.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Robot arm global variables (moved here before function definitions)
// 使用MIT_Motor.cpp中已正确初始化的全局motor_instances数组
extern MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN];

// 确保joint_motor_list指向正确初始化的电机实例
static MotorInstance* joint_motor_list[JOINT_MOTOR_COUNT] = {
    &motor_instances[0][0],  // CAN1, Motor1 (can_id=1, motor_id=1)
    &motor_instances[0][1],  // CAN1, Motor2 (can_id=1, motor_id=2)  
    &motor_instances[0][2],  // CAN1, Motor3 (can_id=1, motor_id=3)
    &motor_instances[0][3],  // CAN1, Motor4 (can_id=1, motor_id=4)
    &motor_instances[0][4],  // CAN1, Motor5 (can_id=1, motor_id=5)
    &motor_instances[0][5]   // CAN1, Motor6 (can_id=1, motor_id=6)
};

// // init kinematics parameters
// static uint32_t current_point = 0;
// static float* raw_joints = NULL;
// static JointAngles input_angles;

// Define predefined joint positions (example data - replace with actual trajectory)
const float predefined_joints[50][6] = {
    // Example trajectory points - replace with your actual robot arm trajectory
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},
    {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f},
    {0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f},
    {0.4f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f},
    {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
    {0.6f, 0.6f, 0.6f, 0.6f, 0.6f, 0.6f},
    {0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f},
    {0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f},
    {0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f},
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    {1.1f, 1.1f, 1.1f, 1.1f, 1.1f, 1.1f},
    {1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f},
    {1.3f, 1.3f, 1.3f, 1.3f, 1.3f, 1.3f},
    {1.4f, 1.4f, 1.4f, 1.4f, 1.4f, 1.4f},
    {1.5f, 1.5f, 1.5f, 1.5f, 1.5f, 1.5f},
    {1.6f, 1.6f, 1.6f, 1.6f, 1.6f, 1.6f},
    {1.7f, 1.7f, 1.7f, 1.7f, 1.7f, 1.7f},
    {1.8f, 1.8f, 1.8f, 1.8f, 1.8f, 1.8f},
    {1.9f, 1.9f, 1.9f, 1.9f, 1.9f, 1.9f},
    {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f},
    {2.1f, 2.1f, 2.1f, 2.1f, 2.1f, 2.1f},
    {2.2f, 2.2f, 2.2f, 2.2f, 2.2f, 2.2f},
    {2.3f, 2.3f, 2.3f, 2.3f, 2.3f, 2.3f},
    {2.4f, 2.4f, 2.4f, 2.4f, 2.4f, 2.4f},
    {2.5f, 2.5f, 2.5f, 2.5f, 2.5f, 2.5f},
    {2.4f, 2.4f, 2.4f, 2.4f, 2.4f, 2.4f},
    {2.3f, 2.3f, 2.3f, 2.3f, 2.3f, 2.3f},
    {2.2f, 2.2f, 2.2f, 2.2f, 2.2f, 2.2f},
    {2.1f, 2.1f, 2.1f, 2.1f, 2.1f, 2.1f},
    {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f},
    {1.9f, 1.9f, 1.9f, 1.9f, 1.9f, 1.9f},
    {1.8f, 1.8f, 1.8f, 1.8f, 1.8f, 1.8f},
    {1.7f, 1.7f, 1.7f, 1.7f, 1.7f, 1.7f},
    {1.6f, 1.6f, 1.6f, 1.6f, 1.6f, 1.6f},
    {1.5f, 1.5f, 1.5f, 1.5f, 1.5f, 1.5f},
    {1.4f, 1.4f, 1.4f, 1.4f, 1.4f, 1.4f},
    {1.3f, 1.3f, 1.3f, 1.3f, 1.3f, 1.3f},
    {1.2f, 1.2f, 1.2f, 1.2f, 1.2f, 1.2f},
    {1.1f, 1.1f, 1.1f, 1.1f, 1.1f, 1.1f},
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    {0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f},
    {0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f},
    {0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f},
    {0.6f, 0.6f, 0.6f, 0.6f, 0.6f, 0.6f},
    {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f},
    {0.4f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f},
    {0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f},
    {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f},
    {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}
};

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _priority) SCHED_TASK_CLASS(Rover, &rover, func, _interval_ticks, _max_time_micros, _priority)

/*
  scheduler table - all regular tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,

    SCHED_TASK(robot_arm_control_loop,100,    200,   1),  // 100Hz control loop  
    SCHED_TASK(robot_arm_interpolation_loop, 5,    500,   2),  // 5Hz trajectory interpolation
    SCHED_TASK(read_radio,             50,    200,   3),
    SCHED_TASK(ahrs_update,           400,    400,   6),
    SCHED_TASK(read_rangefinders,      50,    200,   9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,      &rover.optflow,          update,         200, 160,  11),
#endif
    SCHED_TASK(update_current_mode,   400,    200,  12),
    SCHED_TASK(set_servos,            400,    200,  15),
    SCHED_TASK_CLASS(AP_GPS,              &rover.gps,              update,         50,  300,  18),
    SCHED_TASK_CLASS(AP_Baro,             &rover.barometer,        update,         10,  200,  21),
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon,           &rover.g2.beacon,        update,         50,  200,  24),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,        &rover.g2.proximity,     update,         50,  200,  27),
#endif
    SCHED_TASK_CLASS(AP_WindVane,         &rover.g2.windvane,      update,         20,  100,  30),
    SCHED_TASK(update_wheel_encoder,   50,    200,  36),
    SCHED_TASK_CLASS(Hall_Can_Backend, Hall_Can_Backend::get_singleton(), Log_Write_Motor, 100, 100, 37), // add log motor data module
    SCHED_TASK(update_compass,         10,    200,  39),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(update_logging1,        10,    200,  45),
    SCHED_TASK(update_logging2,        10,    200,  48),
#endif
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_receive,                    400,    500,  51),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_send,                       400,   1000,  54),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_mode_switch,        7,    200,  57),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_aux_all,           10,    200,  60),
    SCHED_TASK_CLASS(AP_BattMonitor,      &rover.battery,          read,           10,  300,  63),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &rover.ServoRelayEvents, update_events,  50,  200,  66),
#endif
#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &rover.g2.gripper,       update,         10,   75,  69),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(update_precland,      400,     50,  70),
#endif
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,              &rover.rpm_sensor,       update,         10,  100,  72),
#endif
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &rover.camera_mount,     update,         50,  200,  75),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &rover.camera,           update,         50,  200,  78),
#endif
    SCHED_TASK(gcs_failsafe_check,     10,    200,  81),
    SCHED_TASK(fence_check,            10,    200,  84),
    SCHED_TASK(ekf_check,              10,    100,  87),
    SCHED_TASK_CLASS(ModeSmartRTL,        &rover.mode_smartrtl,    save_position,   3,  200,  90),
    SCHED_TASK(one_second_loop,         1,   1500,  96),
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,          &rover.g2.sprayer,       update,          3,  90,  99),
#endif
    SCHED_TASK(compass_save,            0.1,  200, 105),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,           &rover.logger,           periodic_tasks, 50,  300, 108),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &rover.ins,              periodic,      400,  200, 111),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,        &rover.scheduler,        update_logging, 0.1, 200, 114),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,           &rover.button,           update,          5,  200, 117),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK(stats_update,            1,    200, 120),
#endif
    SCHED_TASK(crash_check,            10,    200, 123),
    SCHED_TASK(cruise_learn_update,    50,    200, 126),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    200, 129),
#endif
};



// Robot arm initialization - called once during startup
void Rover::robot_arm_init()
{
    if (arm_initialized) {
        return;
    }
    
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        MotorInstance* m = joint_motor_list[i];
        m->enabled = true;
        m->mode = CTRL_MODE_POSITION;
        m->first_command = true;
        m->type = MOTOR_TYPE_MIT;
        memset(&m->queue, 0, sizeof(PositionQueue));
        
        // 验证电机ID是否正确设置 (调试输出，可在调试后删除)
        #ifdef ARDUPILOT_BUILD
        hal.console->printf("Motor %d: CAN_ID=%d, Motor_ID=%d\n", 
                           i, m->can_id, m->motor_id);
        #endif
    }
    
    
    // Initialize trajectory interpolation system
    MIT_Motor::init_trajectory_interpolation();
    
    // // Initialize control state
    // arm_control_counter = 0;
    // arm_t_counter = 0;
    // arm_current_point = 0;
    // debug_counter = 0;
    
    // // Clear debug buffers
    // memset(debug_buffer, 0, sizeof(debug_buffer));
    // memset(real_buffer, 0, sizeof(real_buffer));
    // memset(pos_history, 0, sizeof(pos_history));
    
    arm_initialized = true;
}

// Fast loop - 400Hz - Main kinematics and trajectory planning
// void Rover::robot_arm_fast_loop()
// {
//     if (!arm_initialized) {
//         robot_arm_init();
//         return;
//     }
    
//     MotorInstance* can1_motor2 = &motor_instances[0][1];
    
//     // Original 500Hz logic - generate trajectory points
//     if(can1_motor2->mode == CTRL_MODE_POSITION && arm_current_point < 50) {
//         // Get target parameters
//         memcpy(input_angles.angles, predefined_joints[arm_current_point], sizeof(predefined_joints[arm_current_point]));
        
//         // Forward calculate end pose
//         if (forward_kinematic(&left_arm_config, &input_angles, &target_pose)) {
//             // Inverse calculate joint angle
//             if (inverse_kinematic(&left_arm_config, &target_pose, &input_angles, &solutions)) {
//                 float weights[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
//                 if (find_optimal_solution(&left_arm_config, &solutions, &input_angles, weights, &ik_result)) {
//                     float motor5_pos = ik_result.angles[0] * (180.0f / M_PI);
//                     queue_push(&can1_motor2->queue, motor5_pos);
//                     pos_history[arm_current_point] = motor5_pos;
//                 }
//             }
//         }
//         arm_current_point++;
//     }
// }

// Control loop - 100Hz - Motor control and trapezoid planning  
void Rover::robot_arm_control_loop()
{
    if (!arm_initialized) {
        return;
    }
    
    // Initialize CAN Tx module if not already done
    static bool tx_initialized = false;
    if (!tx_initialized) {
        // 初始化发送队列和处理器 - 在主控制循环中初始化，负责发送电机控制指令
        CAN_Robot_Tx_Queue::init();
        CAN_Robot_Tx_Process::init();
        tx_initialized = true;
    }

    // Process CAN Rx messages first
    process_can_rx_messages();

    // 100Hz motor control - 从稠密队列获取插值后的轨迹点
    float trajectory_point[6];
    bool has_trajectory = MIT_Motor::get_next_trajectory_point(trajectory_point);
    
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        MotorInstance* m = joint_motor_list[i];
        
        if (has_trajectory) {
            // 直接使用稠密矩阵中的轨迹点作为目标值
            m->target_value = trajectory_point[i];
        } 
        // else {
        //     // 如果没有轨迹数据，使用调试值（保持原来的逻辑作为备用）
        //     switch (i) {
        //         case 0:
        //             m->target_value = -3.2f + debug_current1;
        //             break;
        //         case 1:
        //             m->target_value = 6.2923f + debug_current2;
        //             break;
        //         case 2:
        //             m->target_value = -60.0574f + debug_current3;
        //             break;
        //         case 3:
        //             m->target_value = 0.9413f + debug_current4;
        //             break;
        //         case 4:
        //             m->target_value = 57.6267f + debug_current5;
        //             break;                    
        //         case 5:
        //             m->target_value = 10.1124f + debug_current6;
        //             break;                                    
        //     }
        // }
        
        // 直接调用电机控制处理函数
        MIT_Motor::MotorControl_Handler(m);
    }
    // grasper motor control 夹爪接口
    // MotorInstance* gm = grasper_motor_list[0];
    // gm->target_value = debug_current7;
    // handle_kegu_motor(gm);

    // 20Hz debug logging
    // static uint32_t t_counter = 0;
    // if (t_counter % 25 == 0) {
    //     if (debug_counter < 599) {
    //         debug_buffer[debug_counter] = joint_motor_list[4]->planner.current_ref;
    //         real_buffer[debug_counter] = debug_position;
    //         debug_counter++;
    //     }            
    // }
    // t_counter = (t_counter + 1) % 50;
}

// 轨迹点添加模式配置
// 可选值：
//   0 = 一次添加全部关键点 (50个点)
//   1 = 批量添加 (每次5个点)
//   2 = 单个添加 (每次1个点)
//   其他数值 = 自定义每次添加的点数
#define TRAJECTORY_ADD_MODE 1           // 添加模式选择
#define CUSTOM_POINTS_PER_BATCH 5       // 自定义批量大小 (当模式>2时使用)

// Robot arm interpolation loop - 5Hz - Trajectory interpolation
void Rover::robot_arm_interpolation_loop()
{
    if (!arm_initialized) {
        return;
    }
    
    // 轨迹插值参数
    const float Ts = 0.01f;        // 采样周期 [s] - 10ms for 100Hz control loop
    const float F = 10.0f;         // 最大关节速度 [deg/s]
    const float ub_a = 100.0f;      // 最大加速度 [deg/s^2]
    
    // 如果稠密队列为空，尝试生成新轨迹
    if (MIT_Motor::is_dense_queue_empty()) {
        // 静态变量，记录稀疏队列的初始化状态
        static bool sparse_queue_initialized = false;
        
        // 第一次运行时，先把所有predefined_joints添加到稀疏队列
        if (!sparse_queue_initialized) {
            // 添加当前电机位置作为起始点
            MIT_Motor::add_current_motor_positions();
            
            // 一次性将所有predefined_joints添加到稀疏队列
            for (uint8_t i = 0; i < 50; i++) {
                float key_point[6];
                for (uint8_t j = 0; j < 6; j++) {
                    key_point[j] = predefined_joints[i][j] * (180.0f / M_PI);
                }
                MIT_Motor::add_key_point(key_point);
            }
            sparse_queue_initialized = true;
            
            #ifdef ARDUPILOT_BUILD
            hal.console->printf("Sparse queue initialized with %d points\n", 
                              MIT_Motor::sparse_queue_count());
            #endif
        }
        
        // 检查稀疏队列中的点总数
        uint8_t total_points_in_queue = MIT_Motor::sparse_queue_count();
        
        if (total_points_in_queue > 0) {
            // 根据配置模式确定本次处理的点数
            uint8_t points_to_process;
            switch (TRAJECTORY_ADD_MODE) {
                case 0:  // 一次处理全部
                    points_to_process = total_points_in_queue;
                    break;
                case 1:  // 批量处理 (5个点)
                    points_to_process = (total_points_in_queue < 5) ? total_points_in_queue : 5;
                    break;
                case 2:  // 单个处理
                    points_to_process = 1;
                    break;
                default: // 自定义批量大小
                    points_to_process = (total_points_in_queue < CUSTOM_POINTS_PER_BATCH) ? 
                                       total_points_in_queue : CUSTOM_POINTS_PER_BATCH;
                    break;
            }
            
            #ifdef ARDUPILOT_BUILD
            hal.console->printf("Processing %d points from sparse queue (total: %d)\n", 
                              points_to_process, total_points_in_queue);
            #endif
            
            // 生成轨迹插值 (会自动从稀疏队列取出points_to_process个点进行处理)
            MIT_Motor::generate_trajectory_interpolation(Ts, F, ub_a, points_to_process);
        } else {
            // 稀疏队列为空，重置初始化状态，准备下一轮
            sparse_queue_initialized = false;
            
            #ifdef ARDUPILOT_BUILD
            hal.console->printf("Sparse queue empty, ready for next trajectory cycle\n");
            #endif
        }
    }
}

// CAN Rx message processing function - called from robot_arm_control_loop
void Rover::process_can_rx_messages()
{
    // Initialize CAN Rx modules if not already done
    static bool rx_initialized = false;
    if (!rx_initialized) {
        // 初始化接收队列和处理器 - 只处理接收相关的组件
        CAN_Robot_Rx_Queue::init();
        CAN_Robot_Rx_Process::init();
        rx_initialized = true;
    }
    
    // Process all messages from the queue using Rx processor
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (rx_processor != nullptr) {
        rx_processor->process_all_rx_messages();
    }
}




void Rover::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Rover::_failsafe_priorities[7];

Rover::Rover(void) :
    AP_Vehicle(),
    param_loader(var_info),
#if HAL_LOGGING_ENABLED
    logger{g.log_bitmask},
#endif
    modes(&g.mode1),
    control_mode(&mode_initializing),
    // Robot arm initialization
    arm_control_counter(0),
    arm_t_counter(0),
    arm_current_point(0),
    arm_initialized(false),
    debug_counter(0)
{
    // Initialize robot arm arrays
    memset(motor_instances, 0, sizeof(motor_instances));
    memset(usart1_buf, 0, sizeof(usart1_buf));
    memset(pos_history, 0, sizeof(pos_history));
    memset(debug_buffer, 0, sizeof(debug_buffer));
    memset(real_buffer, 0, sizeof(real_buffer));
}

#if AP_SCRIPTING_ENABLED
// set target location (for use by scripting)
bool Rover::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_desired_location(target_loc);
}

// set target velocity (for use by scripting)
bool Rover::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // convert vector length into speed
    const float target_speed_m = safe_sqrt(sq(vel_ned.x) + sq(vel_ned.y));

    // convert vector direction to target yaw
    const float target_yaw_cd = degrees(atan2f(vel_ned.y, vel_ned.x)) * 100.0f;

    // send target heading and speed
    mode_guided.set_desired_heading_and_speed(target_yaw_cd, target_speed_m);

    return true;
}

// set steering and throttle (-1 to +1) (for use by scripting)
bool Rover::set_steering_and_throttle(float steering, float throttle)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // set steering and throttle
    mode_guided.set_steering_and_throttle(steering, throttle);
    return true;
}

// get steering and throttle (-1 to +1) (for use by scripting)
bool Rover::get_steering_and_throttle(float& steering, float& throttle)
{
    steering = g2.motors.get_steering() / 4500.0;
    throttle = g2.motors.get_throttle() * 0.01;
    return true;
}

// set desired turn rate (degrees/sec) and speed (m/s). Used for scripting
bool Rover::set_desired_turn_rate_and_speed(float turn_rate, float speed)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }

    // set turn rate and speed. Turn rate is expected in centidegrees/s and speed in meters/s
    mode_guided.set_desired_turn_rate_and_speed(turn_rate * 100.0f, speed);
    return true;
}

// set desired nav speed (m/s). Used for scripting.
bool Rover::set_desired_speed(float speed)
{
    return control_mode->set_desired_speed(speed);
}

// get control output (for use in scripting)
// returns true on success and control_value is set to a value in the range -1 to +1
bool Rover::get_control_output(AP_Vehicle::ControlOutput control_output, float &control_value)
{
    switch (control_output) {
    case AP_Vehicle::ControlOutput::Roll:
        control_value = constrain_float(g2.motors.get_roll(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Pitch:
        control_value = constrain_float(g2.motors.get_pitch(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Walking_Height:
        control_value = constrain_float(g2.motors.get_walking_height(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Throttle:
        control_value = constrain_float(g2.motors.get_throttle() / 100.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Yaw:
        control_value = constrain_float(g2.motors.get_steering() / 4500.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Lateral:
        control_value = constrain_float(g2.motors.get_lateral() / 100.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::MainSail:
        control_value = constrain_float(g2.motors.get_mainsail() / 100.0f, -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::WingSail:
        control_value = constrain_float(g2.motors.get_wingsail() / 100.0f, -1.0f, 1.0f);
        return true;
    default:
        return false;
    }
    return false;
}

// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Rover::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Rover::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (control_mode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Rover::nav_script_time_done(uint16_t id)
{
    if (control_mode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
#endif // AP_SCRIPTING_ENABLED

#if STATS_ENABLED == ENABLED
/*
  update AP_Stats
*/
void Rover::stats_update(void)
{
    g2.stats.set_flying(g2.motors.active());
    g2.stats.update();
}
#endif


// update AHRS system
void Rover::ahrs_update()
{
    arming.update_soft_armed();

    // AHRS may use movement to calculate heading
    update_ahrs_flyforward();

    ahrs.update();

    // update position
    have_position = ahrs.get_location(current_loc);

    // set home from EKF if necessary and possible
    if (!ahrs.home_is_set()) {
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = velocity.xy().length();
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ground_speed = ahrs.groundspeed();
    }

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }

    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
#endif
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
    if (g.fs_gcs_enabled == FS_GCS_DISABLED) {
        // gcs failsafe disabled
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        // we've never seen the GCS, so we never failsafe for not seeing it
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    const bool do_failsafe = last_gcs_update_ms >= gcs_timeout_ms ? true : false;

    failsafe_trigger(FAILSAFE_EVENT_GCS, "GCS", do_failsafe);
}

#if HAL_LOGGING_ENABLED
/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_THR)) {
        Log_Write_Throttle();
#if AP_BEACON_ENABLED
        g2.beacon.log();
#endif
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        if (g2.pos_control.is_active()) {
            g2.pos_control.write_log();
            logger.Write_PID(LOG_PIDN_MSG, g2.pos_control.get_vel_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, g2.pos_control.get_vel_pid().get_pid_info_y());
        }
    }

#if HAL_PROXIMITY_ENABLED
    if (should_log(MASK_LOG_RANGEFINDER)) {
        g2.proximity.log();
    }
#endif
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        Log_Write_Steering();
    }

    if (should_log(MASK_LOG_RC)) {
        Log_Write_RC();
        g2.wheel_encoder.Log_Write();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_Vibration();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

/*
  once a second events
 */
void Rover::one_second_loop(void)
{
    set_control_channels();

    // cope with changes to aux functions
    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed();
    AP_Notify::flags.flying = hal.util->get_soft_armed();

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // attempt to update home position and baro calibration if not armed:
    if (!hal.util->get_soft_armed()) {
        update_home();
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());

    // send latest param values to wp_nav
    g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
    g2.pos_control.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
    g2.wheel_rate_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
}

void Rover::update_current_mode(void)
{
    // check for emergency stop
    if (SRV_Channels::get_emergency_stop()) {
        // relax controllers, motor stopping done at output level
        g2.attitude_control.relax_I();
    }

    control_mode->update();
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Rover::send_nav_controller_output()
    if (!rover.control_mode->is_autopilot_mode()) {
        return false;
    }
    distance = control_mode->get_distance_to_destination();
    return true;
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Rover::send_nav_controller_output()
    if (!rover.control_mode->is_autopilot_mode()) {
        return false;
    }
    bearing = control_mode->wp_bearing();
    return true;
}

// vehicle specific waypoint info helpers
bool Rover::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Rover::send_nav_controller_output()
    if (!rover.control_mode->is_autopilot_mode()) {
        return false;
    }
    xtrack_error = control_mode->crosstrack_error();
    return true;
}


Rover rover;
AP_Vehicle& vehicle = rover;

AP_HAL_MAIN_CALLBACKS(&rover);
