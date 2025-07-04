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
#include <CAN_Robot_interpolation/TrajectoryInterpolator.h>

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include "AP_Gripper/AP_Gripper.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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



// 使用MIT_Motor.cpp中已正确初始化的全局motor_instances数组
extern MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN];

// 双机械臂电机实例配置
// ARM1: CAN0, Motors 1-6 + Gripper (Motor 7)  
// ARM2: CAN0, Motors 8-13 + Gripper (Motor 14)
// 数组索引映射: motor_instances[can_id][array_index].motor_id
// ARM1关节: [0][0-5].motor_id = 1-6
// ARM1夹爪: [0][6].motor_id = 7  
// ARM2关节: [0][7-12].motor_id = 8-13
// ARM2夹爪: [0][13].motor_id = 14
MotorInstance* arm_motor_list[ARM_COUNT][JOINT_MOTOR_COUNT] = {
    // ARM 1 (关节电机1-6)
    {
        &motor_instances[0][0],  // CAN0, 数组索引0, motor_id=1
        &motor_instances[0][1],  // CAN0, 数组索引1, motor_id=2  
        &motor_instances[0][2],  // CAN0, 数组索引2, motor_id=3
        &motor_instances[0][3],  // CAN0, 数组索引3, motor_id=4
        &motor_instances[0][4],  // CAN0, 数组索引4, motor_id=5
        &motor_instances[0][5]   // CAN0, 数组索引5, motor_id=6
    },
    // ARM 2 (关节电机8-13)
    {
        &motor_instances[0][7],  // CAN0, 数组索引7, motor_id=8
        &motor_instances[0][8],  // CAN0, 数组索引8, motor_id=9
        &motor_instances[0][9],  // CAN0, 数组索引9, motor_id=10
        &motor_instances[0][10], // CAN0, 数组索引10, motor_id=11
        &motor_instances[0][11], // CAN0, 数组索引11, motor_id=12
        &motor_instances[0][12]  // CAN0, 数组索引12, motor_id=13
    }
};

// 双夹爪电机实例（改为全局变量）
MotorInstance* gripper_motors[GRIPPER_COUNT] = {
    &motor_instances[0][6],   // ARM1夹爪: CAN0, 数组索引6, motor_id=7
    &motor_instances[0][13]   // ARM2夹爪: CAN0, 数组索引13, motor_id=14
};

// 双臂控制状态变量（改为全局变量）
bool arms_initialized[ARM_COUNT] = {false, false};
bool grippers_initialized[GRIPPER_COUNT] = {false, false};

// 双臂轨迹插值器
TrajectoryInterpolator arm_interpolators[ARM_COUNT];

// 保持向后兼容的全局变量（指向ARM1）
MotorInstance** joint_motor_list = arm_motor_list[0];
MotorInstance* gripper_motor = gripper_motors[0];

// 新增：双夹爪外部控制命令全局变量
uint8_t kegu_external_commands[2] = {0, 0};        // 外部CAN命令 [0]=ARM1, [1]=ARM2
bool kegu_external_commands_received[2] = {false, false}; // 命令接收标志 [0]=ARM1, [1]=ARM2  
uint32_t kegu_command_timestamps[2] = {0, 0};      // 命令时间戳 [0]=ARM1, [1]=ARM2

// 新增：双夹爪状态数组的全局定义（与Rover类中的成员变量对应）
float gripper_positions[2] = {0.0f, 0.0f};       // 全局夹爪位置数组
float gripper_velocities[2] = {0.0f, 0.0f};      // 全局夹爪速度数组  
float gripper_currents[2] = {0.0f, 0.0f};        // 全局夹爪电流数组

// Robot arm initialization - called once during startup
void Rover::robot_arm_init()
{
    if (arm_initialized) {
        return;
    }
    
    // Initialize motor instances array first
    MIT_Motor::init_motor_instances();
    
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        MotorInstance* m = joint_motor_list[i];
        m->enabled = true;
        m->mode = CTRL_MODE_POSITION;
        m->first_command = true;
        m->type = MOTOR_TYPE_MIT;
        m->queue = PositionQueue(); // Use assignment instead of memset for non-trivial types
        
        // 验证电机ID是否正确设置 (调试输出，可在调试后删除)
        #ifdef ARDUPILOT_BUILD
        hal.console->printf("Motor %d: CAN_ID=%d, Motor_ID=%d\n", 
                           i, m->can_id, m->motor_id);
        #endif
    }
    
    // 初始化夹爪电机 - 使用专用KEGU初始化函数
    
    
    arm_initialized = true;
}

// KEGU电机专用初始化函数
void Rover::kegu_motor_init()
{
    // KEGU电机需要特殊的初始化序列: INIT -> ENABLE -> READY
    gripper_motor->enabled = true;
    gripper_motor->mode = CTRL_MODE_INIT;     // KEGU电机先进行初始化
    gripper_motor->first_command = true;
    gripper_motor->type = MOTOR_TYPE_KEGU;    // 夹爪电机是KEGU类型
    gripper_motor->target_value = 0.0f;       // 初始化参数
    gripper_motor->queue = PositionQueue();
    
    #ifdef ARDUPILOT_BUILD
    hal.console->printf("KEGU Gripper Motor Config: CAN_ID=%d, Motor_ID=%d, Type=KEGU\n", 
                       gripper_motor->can_id, gripper_motor->motor_id);
    hal.console->printf("KEGU Init sequence starting: INIT command...\n");
    #endif
    
    // 步骤1: 发送KEGU电机总线启动指令 (INIT)
    MIT_Motor::MotorControl_Handler(gripper_motor);
    gripper_initialized = true;
}

// 定义CAN2轨迹数据接收相关的常量
#define CAN2_TRAJECTORY_CMD_ID 0x100  // CAN2轨迹命令ID
#define CAN2_TRAJECTORY_DATA_LENGTH 8 // 6个关节位置数据 + 2字节控制信息

// 全局轨迹数据接收状态 (在头文件中声明，这里定义)
CAN2TrajectoryData latest_trajectory_data;
bool trajectory_data_received = false;
uint32_t last_trajectory_receive_time = 0;

// 双臂独立的轨迹数据队列
TrajectoryDataQueue trajectory_queues[2];  // [0]=ARM1, [1]=ARM2

// KEGU夹爪电机控制变量 (定义在函数外面)
bool kegu_control_enabled = false;        // 不使用static，允许外部访问
float kegu_target_current = 0.0f;         // 不使用static，允许外部访问

// KEGU外部控制命令全局变量
uint8_t kegu_external_command = 0;        // 外部CAN命令 (ARM1夹爪，向后兼容)
bool kegu_external_command_received = false; // 命令接收标志 (ARM1夹爪，向后兼容)
uint32_t kegu_command_timestamp = 0;      // 命令时间戳 (ARM1夹爪，向后兼容)



void Rover::robot_arm_control_loop()
{
    // === 1. 初始化阶段 ===
    static bool rx_initialized = false;
    static bool tx_initialized = false;
    
    if (!rx_initialized) {
        CAN_Robot_Rx_Queue::init();
        CAN_Robot_Rx_Process::init();
        rx_initialized = true;
    }
    
    if (!tx_initialized) {
        CAN_Robot_Tx_Queue::init();
        CAN_Robot_Tx_Process::init();
        tx_initialized = true;
    }
    
    // === 2. CAN消息处理 ===
    CAN_Robot_Rx_Process* rx_processor = CAN_Robot_Rx_Process::get_singleton();
    if (rx_processor != nullptr) {
        rx_processor->process_all_rx_messages();
    }
    
    // === 3. 双臂系统初始化 ===
    static bool dual_arm_init_complete = false;
    if (!dual_arm_init_complete) {
        dual_arm_init();
        
        // 检查是否所有臂都已初始化
        bool all_arms_ready = true;
        for (uint8_t i = 0; i < ARM_COUNT; i++) {
            if (!arms_initialized[i]) {
                all_arms_ready = false;
                break;
            }
        }
        
        if (all_arms_ready) {
            dual_arm_init_complete = true;
            AP::logger().Write_MessageF("DUAL_ARM: All %d arms initialized successfully", ARM_COUNT);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Dual-arm system ready");
        } else {
            return; // 等待初始化完成
        }
    }
    
    // === 4. KEGU夹爪初始化命令发送 ===
    static bool kegu_init_complete = false;
    static uint32_t kegu_init_start_time = 0;
    static bool kegu_init_sent[GRIPPER_COUNT] = {false, false}; // 跟踪每个夹爪是否已发送初始化命令
    
    if (dual_arm_init_complete && !kegu_init_complete) {
        uint32_t now_ms = AP_HAL::millis();
        
        // 首次进入，记录开始时间
        if (kegu_init_start_time == 0) {
            kegu_init_start_time = now_ms;
            AP::logger().Write_MessageF("KEGU_INIT: Starting gripper initialization sequence");
        }
        
        // 为每个夹爪发送初始化命令（只发送一次）
        for (uint8_t gripper_id = 0; gripper_id < GRIPPER_COUNT; gripper_id++) {
            if (!grippers_initialized[gripper_id] || kegu_init_sent[gripper_id]) {
                continue; // 跳过未初始化的夹爪或已发送命令的夹爪
            }
            
            MotorInstance* gripper = gripper_motors[gripper_id];
            
            // 发送总线启动指令 (INIT) - 只发送一次
            gripper->mode = CTRL_MODE_INIT;
            gripper->target_value = 0.0f;
            MIT_Motor::MotorControl_Handler(gripper);
            kegu_init_sent[gripper_id] = true; // 标记已发送
            hal.scheduler->delay(100); // 延时100ms
            
            #ifdef ARDUPILOT_BUILD
            hal.console->printf("ARM%d Gripper: Sending INIT command (CAN_ID=%d, Motor_ID=%d) - ONCE\n", 
                               gripper_id+1, gripper->can_id, gripper->motor_id);
            #endif
            AP::logger().Write_MessageF("ARM%d_GRIPPER: INIT command sent (motor_id=%d)", gripper_id+1, gripper->motor_id);
        }
        
        // 等待100ms后标记初始化完成
        if (now_ms - kegu_init_start_time > 100) {
            kegu_init_complete = true;
            uint32_t init_duration = now_ms - kegu_init_start_time;
            AP::logger().Write_MessageF("KEGU_INIT: All %d grippers ready (took %ums)", GRIPPER_COUNT, init_duration);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KEGU grippers initialized");
        } else if (now_ms - kegu_init_start_time > 5000) {
            // 超时保护 - 5秒后强制完成
            kegu_init_complete = true;
            AP::logger().Write_MessageF("KEGU_INIT: Timeout - forcing completion");
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "KEGU gripper init timeout");
        }
        
        if (!kegu_init_complete) {
            return; // 等待夹爪初始化完成
        }
    }
    
    // === 5. 性能监控开始 ===
    uint32_t loop_start_time_us = AP_HAL::micros();
    
    // === 6. 批量更新所有机械臂状态 ===
    for (uint8_t arm_id = 0; arm_id < ARM_COUNT; arm_id++) {
        update_arm_status(arm_id);
    }
    
    // === 7. 批量控制所有机械臂 ===
    for (uint8_t arm_id = 0; arm_id < ARM_COUNT; arm_id++) {
        control_arm_motors(arm_id);
    }
    
    // === 8. 双夹爪智能控制 ===
    MIT_Motor::process_dual_gripper_control();
    
    // === 9. 性能监控和日志记录 ===
    uint32_t loop_execution_time_us = AP_HAL::micros() - loop_start_time_us;
    
    // 性能警告（如果超过预算的80%）
    static uint32_t last_performance_warning_ms = 0;
    if (loop_execution_time_us > 160) { // 200μs的80%
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_performance_warning_ms > 1000) { // 每秒最多一次警告
            AP::logger().Write_MessageF("DUAL_ARM_PERF: Loop time %uμs > 160μs threshold", loop_execution_time_us);
            last_performance_warning_ms = now_ms;
        }
    }
    
    #if HAL_LOGGING_ENABLED
    // 低频率日志记录（5Hz，避免影响性能）
    static uint32_t last_log_time_ms = 0;
    uint32_t log_now_ms = AP_HAL::millis();
    if (log_now_ms - last_log_time_ms >= 200) { // 每200ms记录一次
        Log_Write_RobotArm1();  // ARM1 Motors 1-3
        Log_Write_RobotArm2();  // ARM1 Motors 4-6
        // TODO: 添加ARM2的日志记录函数
        Log_Write_GripperMotor(); // 所有夹爪
        Log_Write_InterpolatedTrajectory(); // 所有轨迹数据
        
        // 记录性能数据
        AP::logger().Write_MessageF("DUAL_ARM_TIMING: Loop=%uμs Budget=200μs Usage=%.1f%%", 
                                   loop_execution_time_us, (loop_execution_time_us * 100.0f) / 200.0f);
        
        last_log_time_ms = log_now_ms;
    }
    #endif
    
    // === 10. 向后兼容性支持 ===
    // gripper_position, gripper_velocity, gripper_current 现在是引用，自动指向gripper_positions[0]等
    // 无需手动更新
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
    debug_counter(0),
    // Initialize references to gripper arrays[0] for backward compatibility
    gripper_position(gripper_positions[0]),
    gripper_velocity(gripper_velocities[0]),
    gripper_current(gripper_currents[0])
{
    // Initialize robot arm arrays
    memset(motor_instances, 0, sizeof(motor_instances));
    memset(usart1_buf, 0, sizeof(usart1_buf));
    memset(pos_history, 0, sizeof(pos_history));
    memset(debug_buffer, 0, sizeof(debug_buffer));
    memset(real_buffer, 0, sizeof(real_buffer));
    
    // Initialize motor status arrays for efficient CAN RX data storage
    memset(motor_positions, 0, sizeof(motor_positions));
    memset(motor_velocities, 0, sizeof(motor_velocities));
    memset(motor_currents, 0, sizeof(motor_currents));
    
    // Initialize gripper arrays
    memset(gripper_positions, 0, sizeof(gripper_positions));
    memset(gripper_velocities, 0, sizeof(gripper_velocities));
    memset(gripper_currents, 0, sizeof(gripper_currents));
    
    // Initialize interpolation tracking arrays
    memset(prev_interpolated_pos, 0, sizeof(prev_interpolated_pos));
    memset(interpolated_velocities, 0, sizeof(interpolated_velocities));
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

//---------------------Gripper Motor Control Function Implementations---------------------
// Public interface functions that call MIT_Motor namespace functions

void Rover::set_gripper_current(float current_value)
{
    MIT_Motor::set_gripper_current(current_value);
}

void Rover::set_gripper_velocity(float velocity_value)
{
    MIT_Motor::set_gripper_velocity(velocity_value);
}

float Rover::get_gripper_position()
{
    return MIT_Motor::get_gripper_position();
}

float Rover::get_gripper_velocity()
{
    return MIT_Motor::get_gripper_velocity();
}

float Rover::get_gripper_current()
{
    return MIT_Motor::get_gripper_current();
}

void Rover::gripper_open(float speed_percentage)
{
    MIT_Motor::gripper_open(speed_percentage);
}

void Rover::gripper_close(float speed_percentage)
{
    MIT_Motor::gripper_close(speed_percentage);
}

void Rover::gripper_stop()
{
    MIT_Motor::gripper_stop();
}

float Rover::get_gripper_target_velocity()
{
    // 直接使用全局的gripper_motor变量
    if (gripper_motor && gripper_motor->mode == CTRL_MODE_VELOCITY) {
        return gripper_motor->target_value;
    }
    return 0.0f;  // 如果不是速度模式，返回0
}

float Rover::get_gripper_target_current()
{
    // 直接使用全局的gripper_motor变量
    if (gripper_motor && gripper_motor->mode == CTRL_MODE_CURRENT) {
        return gripper_motor->target_value;
    }
    return 0.0f;  // 如果不是电流模式，返回0
}

void Rover::set_kegu_control_enabled(bool enabled)
{
    // 访问外部定义的静态变量
    extern bool kegu_control_enabled;
    extern float kegu_target_current;
    
    kegu_control_enabled = enabled;
    
    if (!enabled) {
        // 禁用控制时，停止电机
        kegu_target_current = 0.0f;
        MIT_Motor::set_gripper_current(0.0f);
    }
    
    #ifdef ARDUPILOT_BUILD
    hal.console->printf("KEGU Control %s\n", enabled ? "ENABLED" : "DISABLED");
    #endif
}

// 新增：双臂初始化函数
void Rover::dual_arm_init()
{
    // 首先初始化电机实例数组
    MIT_Motor::init_motor_instances();
    
    // 为每个机械臂初始化电机
    for (uint8_t arm_id = 0; arm_id < ARM_COUNT; arm_id++) {
        if (arms_initialized[arm_id]) {
            continue;
        }
        
        for (uint8_t joint_id = 0; joint_id < JOINT_MOTOR_COUNT; joint_id++) {
            MotorInstance* m = arm_motor_list[arm_id][joint_id];
            m->enabled = true;
            m->mode = CTRL_MODE_POSITION;
            m->first_command = true;
            m->type = MOTOR_TYPE_MIT;
            m->queue = PositionQueue();
            
            #ifdef ARDUPILOT_BUILD
            hal.console->printf("ARM%d Motor%d: CAN_ID=%d, Motor_ID=%d\n", 
                               arm_id+1, joint_id+1, m->can_id, m->motor_id);
            #endif
        }
        
        // 初始化对应的轨迹插值器
        arm_interpolators[arm_id].init(0.01f, 10.0f, 100.0f);
        arms_initialized[arm_id] = true;
        
        AP::logger().Write_MessageF("ARM%d: Initialized %d joint motors", arm_id+1, JOINT_MOTOR_COUNT);
    }
    
    // 初始化夹爪电机
    for (uint8_t gripper_id = 0; gripper_id < GRIPPER_COUNT; gripper_id++) {
        if (grippers_initialized[gripper_id]) {
            continue;
        }
        
        MotorInstance* gripper = gripper_motors[gripper_id];
        gripper->enabled = true;
        gripper->mode = CTRL_MODE_INIT;
        gripper->first_command = true;
        gripper->type = MOTOR_TYPE_KEGU;
        gripper->target_value = 0.0f;
        gripper->queue = PositionQueue();
        
        grippers_initialized[gripper_id] = true;
        
        #ifdef ARDUPILOT_BUILD
        hal.console->printf("ARM%d Gripper: CAN_ID=%d, Motor_ID=%d\n", 
                           gripper_id+1, gripper->can_id, gripper->motor_id);
        #endif
        
        AP::logger().Write_MessageF("ARM%d: KEGU gripper initialized", gripper_id+1);
    }
}

// 新增：单个机械臂状态更新
void Rover::update_arm_status(uint8_t arm_id)
{
    if (arm_id >= ARM_COUNT) return;
    
    // 更新关节电机状态
    for (uint8_t joint_id = 0; joint_id < JOINT_MOTOR_COUNT; joint_id++) {
        MotorInstance* m = arm_motor_list[arm_id][joint_id];
        uint8_t global_motor_idx = arm_id * JOINT_MOTOR_COUNT + joint_id;
        
        motor_positions[global_motor_idx] = MIT_Motor::get_motor_position(m->can_id, m->motor_id);
        motor_velocities[global_motor_idx] = MIT_Motor::get_motor_velocity(m->can_id, m->motor_id);
        motor_currents[global_motor_idx] = MIT_Motor::get_motor_current(m->can_id, m->motor_id);
        
        // 更新last_position
        if (!isnan(motor_positions[global_motor_idx])) {
            m->last_position = motor_positions[global_motor_idx];
        }
    }
    
    // 更新夹爪状态
    if (arm_id < GRIPPER_COUNT) {
        gripper_positions[arm_id] = MIT_Motor::get_gripper_position_by_id(arm_id);
        gripper_velocities[arm_id] = MIT_Motor::get_gripper_velocity_by_id(arm_id); 
        gripper_currents[arm_id] = MIT_Motor::get_gripper_current_by_id(arm_id);
    }
}

// 新增：单个机械臂轨迹插值处理
void Rover::process_arm_interpolation(uint8_t arm_id)
{
    if (arm_id >= ARM_COUNT) return;
    
    // 检查插值器是否已初始化
    static bool interpolators_initialized[ARM_COUNT] = {false, false};
    
    if (!interpolators_initialized[arm_id]) {
        // 获取当前电机位置进行初始化
        float current_positions[JOINT_MOTOR_COUNT];
        bool all_positions_valid = true;
        
        for (uint8_t joint_id = 0; joint_id < JOINT_MOTOR_COUNT; joint_id++) {
            MotorInstance* m = arm_motor_list[arm_id][joint_id];
            current_positions[joint_id] = MIT_Motor::get_motor_position(m->can_id, m->motor_id);
            
            if (isnan(current_positions[joint_id]) || fabsf(current_positions[joint_id]) > 720.0f) {
                all_positions_valid = false;
                break;
            }
        }
        
        if (!all_positions_valid) {
            return; // 等待有效位置数据
        }
        
        // 设置初始位置
        arm_interpolators[arm_id].set_initial_position(current_positions);
        interpolators_initialized[arm_id] = true;
        
        #ifdef ARDUPILOT_BUILD
        hal.console->printf("ARM%d: Trajectory interpolator initialized\n", arm_id+1);
        #endif
    }
    
    // 检查是否有新的轨迹数据需要处理
    CAN2TrajectoryData new_trajectory;
    if (trajectory_queues[arm_id].pop(new_trajectory)) {
        // 发现新轨迹点，设置到插值器
        arm_interpolators[arm_id].add_trajectory_point(new_trajectory.joint_positions);
    }
    
    // 执行轨迹插值
    float interpolated_pos[JOINT_MOTOR_COUNT];
    arm_interpolators[arm_id].update(interpolated_pos);
    
    // 计算插值速度
    for (uint8_t joint_id = 0; joint_id < JOINT_MOTOR_COUNT; joint_id++) {
        interpolated_velocities[arm_id][joint_id] = 
            (interpolated_pos[joint_id] - prev_interpolated_pos[arm_id][joint_id]) / 0.01f;
        prev_interpolated_pos[arm_id][joint_id] = interpolated_pos[joint_id];
    }
    
    // 发送控制指令到电机
    for (uint8_t joint_id = 0; joint_id < JOINT_MOTOR_COUNT; joint_id++) {
        MotorInstance* m = arm_motor_list[arm_id][joint_id];
        m->target_value = interpolated_pos[joint_id];
        MIT_Motor::MotorControl_Handler(m);
    }
}

// 新增：单个机械臂电机控制
void Rover::control_arm_motors(uint8_t arm_id)
{
    if (arm_id >= ARM_COUNT || !arms_initialized[arm_id]) {
        return;
    }
    
    // 处理轨迹插值和电机控制
    process_arm_interpolation(arm_id);
    
    // 处理夹爪控制（如果对应夹爪存在且已初始化）
    if (arm_id < GRIPPER_COUNT && grippers_initialized[arm_id]) {
        MotorInstance* gripper = gripper_motors[arm_id];
        
        // 夹爪模式管理（简化版本，实际可能需要独立状态机）
        static bool gripper_mode_switched[GRIPPER_COUNT] = {false, false};
        
        if (!gripper_mode_switched[arm_id] && gripper->mode == CTRL_MODE_INIT) {
            gripper->mode = CTRL_MODE_VELOCITY;
            gripper_mode_switched[arm_id] = true;
            
            AP::logger().Write_MessageF("ARM%d_GRIPPER: Switched to velocity control", arm_id+1);
        }
        
        if (gripper_mode_switched[arm_id]) {
            // 基本夹爪控制逻辑（可根据需要扩展）
            static float gripper_target_velocities[GRIPPER_COUNT] = {0.0f, 0.0f};
            
            // 电流保护
            if (fabsf(gripper_currents[arm_id]) > 200.0f) {
                gripper_target_velocities[arm_id] = 0.0f;
            }
            
            gripper->target_value = gripper_target_velocities[arm_id];
            MIT_Motor::MotorControl_Handler(gripper);
        }
    }
}

