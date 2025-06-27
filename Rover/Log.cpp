#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>


#if HAL_LOGGING_ENABLED

// Write an attitude packet
void Rover::Log_Write_Attitude()
{
    float desired_pitch_cd = degrees(g2.attitude_control.get_desired_pitch()) * 100.0f;
    const Vector3f targets(0.0f, desired_pitch_cd, 0.0f);

    ahrs.Write_Attitude(targets);

    AP::ahrs().Log_Write();

    // log steering rate controller
    logger.Write_PID(LOG_PIDS_MSG, g2.attitude_control.get_steering_rate_pid().get_pid_info());
    logger.Write_PID(LOG_PIDA_MSG, g2.attitude_control.get_throttle_speed_pid_info());

    // log pitch control for balance bots
    if (is_balancebot()) {
        logger.Write_PID(LOG_PIDP_MSG, g2.attitude_control.get_pitch_to_throttle_pid().get_pid_info());
    }

    // log heel to sail control for sailboats
    if (rover.g2.sailboat.sail_enabled()) {
        logger.Write_PID(LOG_PIDR_MSG, g2.attitude_control.get_sailboat_heel_pid().get_pid_info());
    }
}

// Write a range finder depth message
void Rover::Log_Write_Depth()
{
    // only log depth on boats
    if (!rover.is_boat() || !rangefinder.has_orientation(ROTATION_PITCH_270)) {
        return;
    }

    // get position
    Location loc;
    IGNORE_RETURN(ahrs.get_location(loc));

    for (uint8_t i=0; i<rangefinder.num_sensors(); i++) {
        const AP_RangeFinder_Backend *s = rangefinder.get_backend(i);
        
        if (s == nullptr || s->orientation() != ROTATION_PITCH_270 || !s->has_data()) {
            continue;
        }

        // check if new sensor reading has arrived
        const uint32_t reading_ms = s->last_reading_ms();
        if (reading_ms == rangefinder_last_reading_ms[i]) {
            continue;
        }
        rangefinder_last_reading_ms[i] = reading_ms;

        float temp_C;
        if (!s->get_temp(temp_C)) {
            temp_C = 0.0f;
        }

        // @LoggerMessage: DPTH
        // @Description: Depth messages on boats with downwards facing range finder
        // @Field: TimeUS: Time since system startup
        // @Field: Inst: Instance
        // @Field: Lat: Latitude 
        // @Field: Lng: Longitude   
        // @Field: Depth: Depth as detected by the sensor
        // @Field: Temp: Temperature

        logger.Write("DPTH", "TimeUS,Inst,Lat,Lng,Depth,Temp",
                            "s#DUmO", "F-GG00", "QBLLff",
                            AP_HAL::micros64(),
                            i,
                            loc.lat,
                            loc.lng,
                            (double)(s->distance()),
                            temp_C);
    }
#if AP_RANGEFINDER_ENABLED
    // send water depth and temp to ground station
    gcs().send_message(MSG_WATER_DEPTH);
#endif
}

// guided mode logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    float wp_bearing;
    float nav_bearing;
    uint16_t yaw;
    float xtrack_error;
};

// Write a navigation tuning packet
void Rover::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : control_mode->get_distance_to_destination(),
        wp_bearing          : control_mode->wp_bearing(),
        nav_bearing         : control_mode->nav_bearing(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        xtrack_error        : control_mode->crosstrack_error()
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_Sail()
{
    // only log sail if present
    if (!rover.g2.sailboat.sail_enabled()) {
        return;
    }

    float wind_dir_tack = logger.quiet_nanf();
    uint8_t current_tack = 0;
    if (rover.g2.windvane.enabled()) {
        wind_dir_tack = degrees(g2.windvane.get_tack_threshold_wind_dir_rad());
        current_tack = uint8_t(g2.windvane.get_current_tack());
    }

// @LoggerMessage: SAIL
// @Description: Sailboat information
// @Field: TimeUS: Time since system startup
// @Field: Tack: Current tack, 0 = port, 1 = starboard
// @Field: TackThr: Apparent wind angle used for tack threshold
// @Field: MainOut: Normalized mainsail output
// @Field: WingOut: Normalized wingsail output
// @Field: MastRotOut: Normalized direct-rotation mast output
// @Field: VMG: Velocity made good (speed at which vehicle is making progress directly towards destination)

    logger.Write("SAIL", "TimeUS,Tack,TackThr,MainOut,WingOut,MastRotOut,VMG",
                        "s-d%%%n", "F000000", "QBfffff",
                        AP_HAL::micros64(),
                        current_tack,
                        (double)wind_dir_tack,
                        (double)g2.motors.get_mainsail(),
                        (double)g2.motors.get_wingsail(),
                        (double)g2.motors.get_mast_rotation(),
                        (double)g2.sailboat.get_VMG());
}

struct PACKED log_Steering {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t steering_in;
    float steering_out;
    float desired_lat_accel;
    float lat_accel;
    float desired_turn_rate;
    float turn_rate;
};

// Write a steering packet
void Rover::Log_Write_Steering()
{
    float lat_accel = logger.quiet_nanf();
    g2.attitude_control.get_lat_accel(lat_accel);
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_us        : AP_HAL::micros64(),
        steering_in        : channel_steer->get_control_in(),
        steering_out       : g2.motors.get_steering(),
        desired_lat_accel  : control_mode->get_desired_lat_accel(),
        lat_accel          : lat_accel,
        desired_turn_rate  : degrees(g2.attitude_control.get_desired_turn_rate()),
        turn_rate          : degrees(ahrs.get_yaw_rate_earth())
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Throttle {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t throttle_in;
    float throttle_out;
    float desired_speed;
    float speed;
    float accel_x;
};

// Write a throttle control packet
void Rover::Log_Write_Throttle()
{
    const Vector3f accel = ins.get_accel();
    float speed = logger.quiet_nanf();
    g2.attitude_control.get_forward_speed(speed);
    struct log_Throttle pkt = {
        LOG_PACKET_HEADER_INIT(LOG_THR_MSG),
        time_us         : AP_HAL::micros64(),
        throttle_in     : channel_throttle->get_control_in(),
        throttle_out    : g2.motors.get_throttle(),
        desired_speed   : g2.attitude_control.get_desired_speed(),
        speed           : speed,
        accel_x         : accel.x
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
}

// Robot Arm Motor Data logging structure - Motors 1-3
struct PACKED log_RobotArm1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float motor1_pos;
    float motor1_vel;
    float motor1_curr;
    float motor2_pos;
    float motor2_vel;
    float motor2_curr;
    float motor3_pos;
    float motor3_vel;
    float motor3_curr;
};

// Robot Arm Motor Data logging structure - Motors 4-6
struct PACKED log_RobotArm2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float motor4_pos;
    float motor4_vel;
    float motor4_curr;
    float motor5_pos;
    float motor5_vel;
    float motor5_curr;
    float motor6_pos;
    float motor6_vel;
    float motor6_curr;
};

// Interpolated Trajectory logging structure
struct PACKED log_InterpolatedTrajectory {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float interp_pos1;
    float interp_pos2;
    float interp_pos3;
    float interp_pos4;
    float interp_pos5;
    float interp_pos6;
    float interp_vel1;
    float interp_vel2;
    float interp_vel3;
    float interp_vel4;
    float interp_vel5;
    float interp_vel6;
};

// Motor Control Commands logging structure - 记录实际发送给电机的控制指令（1-3）
struct PACKED log_MotorControlCmd1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float cmd_pos1;
    float cmd_pos2;
    float cmd_pos3;
    float feedback_pos1;
    float feedback_pos2;
    float feedback_pos3;
};
// Motor Control Commands logging structure - 记录实际发送给电机的控制指令（4-6）
struct PACKED log_MotorControlCmd2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float cmd_pos4;
    float cmd_pos5;
    float cmd_pos6;
    float feedback_pos4;
    float feedback_pos5;
    float feedback_pos6;
};

// Write robot arm motor data - Motors 1-3
void Rover::Log_Write_RobotArm1()
{
    // 添加调试输出
    #ifdef ARDUPILOT_BUILD
    hal.console->printf("ROB1: Pos[%.2f,%.2f,%.2f] Vel[%.2f,%.2f,%.2f] Curr[%.2f,%.2f,%.2f]\n",
                       motor_positions[0], motor_positions[1], motor_positions[2],
                       motor_velocities[0], motor_velocities[1], motor_velocities[2],
                       motor_currents[0], motor_currents[1], motor_currents[2]);
    #endif
    
    struct log_RobotArm1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ROBOT_ARM_MSG),
        time_us         : AP_HAL::micros64(),
        motor1_pos      : motor_positions[0],
        motor1_vel      : motor_velocities[0],
        motor1_curr     : motor_currents[0],
        motor2_pos      : motor_positions[1],
        motor2_vel      : motor_velocities[1],
        motor2_curr     : motor_currents[1],
        motor3_pos      : motor_positions[2],
        motor3_vel      : motor_velocities[2],
        motor3_curr     : motor_currents[2]
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write robot arm motor data - Motors 4-6
void Rover::Log_Write_RobotArm2()
{
    // 添加调试输出
    #ifdef ARDUPILOT_BUILD
    hal.console->printf("ROB2: Pos[%.2f,%.2f,%.2f] Vel[%.2f,%.2f,%.2f] Curr[%.2f,%.2f,%.2f]\n",
                       motor_positions[3], motor_positions[4], motor_positions[5],
                       motor_velocities[3], motor_velocities[4], motor_velocities[5],
                       motor_currents[3], motor_currents[4], motor_currents[5]);
    #endif
    
    struct log_RobotArm2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ROBOT_ARM2_MSG),
        time_us         : AP_HAL::micros64(),
        motor4_pos      : motor_positions[3],
        motor4_vel      : motor_velocities[3],
        motor4_curr     : motor_currents[3],
        motor5_pos      : motor_positions[4],
        motor5_vel      : motor_velocities[4],
        motor5_curr     : motor_currents[4],
        motor6_pos      : motor_positions[5],
        motor6_vel      : motor_velocities[5],
        motor6_curr     : motor_currents[5]
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write interpolated trajectory data
void Rover::Log_Write_InterpolatedTrajectory()
{
    struct log_InterpolatedTrajectory pkt = {
        LOG_PACKET_HEADER_INIT(LOG_INTERP_TRAJ_MSG),
        time_us         : AP_HAL::micros64(),
        interp_pos1     : prev_interpolated_pos[0],
        interp_pos2     : prev_interpolated_pos[1],
        interp_pos3     : prev_interpolated_pos[2],
        interp_pos4     : prev_interpolated_pos[3],
        interp_pos5     : prev_interpolated_pos[4],
        interp_pos6     : prev_interpolated_pos[5],
        interp_vel1     : interpolated_velocities[0],
        interp_vel2     : interpolated_velocities[1],
        interp_vel3     : interpolated_velocities[2],
        interp_vel4     : interpolated_velocities[3],
        interp_vel5     : interpolated_velocities[4],
        interp_vel6     : interpolated_velocities[5]
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write motor control commands data - 1-3号电机
void Rover::Log_Write_MotorControlCmd1()
{
    
    struct log_MotorControlCmd1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MCMD1_MSG),
        time_us         : AP_HAL::micros64(),
        cmd_pos1        : prev_interpolated_pos[0],  // 使用插值位置作为命令位置
        cmd_pos2        : prev_interpolated_pos[1],
        cmd_pos3        : prev_interpolated_pos[2],
        feedback_pos1   : motor_positions[0],
        feedback_pos2   : motor_positions[1],
        feedback_pos3   : motor_positions[2]
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}
// Write motor control commands data - 4-6号电机
void Rover::Log_Write_MotorControlCmd2()
{
  
    struct log_MotorControlCmd2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MCMD2_MSG),
        time_us         : AP_HAL::micros64(),
        cmd_pos4        : prev_interpolated_pos[3],  // 使用插值位置作为命令位置
        cmd_pos5        : prev_interpolated_pos[4],
        cmd_pos6        : prev_interpolated_pos[5],
        feedback_pos4   : motor_positions[3],
        feedback_pos5   : motor_positions[4],
        feedback_pos6   : motor_positions[5]
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode((uint8_t)control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information

const LogStructure Rover::log_structure[] = {
    LOG_COMMON_STRUCTURES,

// @LoggerMessage: THR
// @Description: Throttle related messages
// @Field: TimeUS: Time since system startup
// @Field: ThrIn: Throttle Input
// @Field: ThrOut: Throttle Output 
// @Field: DesSpeed: Desired speed 
// @Field: Speed: Actual speed
// @Field: AccX: Acceleration

    { LOG_THR_MSG, sizeof(log_Throttle),
      "THR", "Qhffff", "TimeUS,ThrIn,ThrOut,DesSpeed,Speed,AccX", "s--nno", "F--000" },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @URL: http://ardupilot.org/rover/docs/navigation.html
// @Field: TimeUS: Time since system startup
// @Field: WpDist: distance to the current navigation waypoint
// @Field: WpBrg: bearing to the current navigation waypoint
// @Field: DesYaw: the vehicle's desired heading
// @Field: Yaw: the vehicle's current heading
// @Field: XTrack: the vehicle's current distance from the current travel segment

    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),
      "NTUN", "QfffHf", "TimeUS,WpDist,WpBrg,DesYaw,Yaw,XTrack", "smhhhm", "F000B0" },
    
// @LoggerMessage: STER
// @Description: Steering related messages
// @Field: TimeUS: Time since system startup
// @Field: SteerIn: Steering input
// @Field: SteerOut: Normalized steering output 
// @Field: DesLatAcc: Desired lateral acceleration
// @Field: LatAcc: Actual lateral acceleration
// @Field: DesTurnRate: Desired turn rate
// @Field: TurnRate: Actual turn rate
    
    { LOG_STEERING_MSG, sizeof(log_Steering),
      "STER", "Qhfffff",   "TimeUS,SteerIn,SteerOut,DesLatAcc,LatAcc,DesTurnRate,TurnRate", "s--ookk", "F--0000" },

// @LoggerMessage: GUIP
// @Description: Guided mode target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
    
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUIP",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },

// @LoggerMessage: ROB
// @Description: Robot Arm Motor Data
// @Field: TimeUS: Time since system startup
// @Field: Motor1Pos: Motor 1 position
// @Field: Motor1Vel: Motor 1 velocity
// @Field: Motor1Curr: Motor 1 current
// @Field: Motor2Pos: Motor 2 position
// @Field: Motor2Vel: Motor 2 velocity
// @Field: Motor2Curr: Motor 2 current
// @Field: Motor3Pos: Motor 3 position
// @Field: Motor3Vel: Motor 3 velocity
// @Field: Motor3Curr: Motor 3 current
// @Field: Motor4Pos: Motor 4 position
// @Field: Motor4Vel: Motor 4 velocity
// @Field: Motor4Curr: Motor 4 current
// @Field: Motor5Pos: Motor 5 position
// @Field: Motor5Vel: Motor 5 velocity
// @Field: Motor5Curr: Motor 5 current
// @Field: Motor6Pos: Motor 6 position
// @Field: Motor6Vel: Motor 6 velocity
// @Field: Motor6Curr: Motor 6 current

    { LOG_ROBOT_ARM_MSG, sizeof(log_RobotArm1),
      "ROB", "Qfffffffff", "TimeUS,M1Pos,M1Vel,M1Curr,M2Pos,M2Vel,M2Curr,M3Pos,M3Vel,M3Curr", "sddAddAddAdd", "F000000000" },

    { LOG_ROBOT_ARM2_MSG, sizeof(log_RobotArm2),
      "ROB2", "Qfffffffff", "TimeUS,M4Pos,M4Vel,M4Curr,M5Pos,M5Vel,M5Curr,M6Pos,M6Vel,M6Curr", "sddAddAddAdd", "F000000000" },

// @LoggerMessage: ITRJ
// @Description: Interpolated Trajectory Data
// @Field: TimeUS: Time since system startup
// @Field: IPos1: Interpolated position motor 1
// @Field: IPos2: Interpolated position motor 2
// @Field: IPos3: Interpolated position motor 3
// @Field: IPos4: Interpolated position motor 4
// @Field: IPos5: Interpolated position motor 5
// @Field: IPos6: Interpolated position motor 6
// @Field: IVel1: Interpolated velocity motor 1
// @Field: IVel2: Interpolated velocity motor 2
// @Field: IVel3: Interpolated velocity motor 3
// @Field: IVel4: Interpolated velocity motor 4
// @Field: IVel5: Interpolated velocity motor 5
// @Field: IVel6: Interpolated velocity motor 6

    { LOG_INTERP_TRAJ_MSG, sizeof(log_InterpolatedTrajectory),
      "ITRJ", "Qffffffffffff", "TimeUS,IPos1,IPos2,IPos3,IPos4,IPos5,IPos6,IVel1,IVel2,IVel3,IVel4,IVel5,IVel6", "sddddddoooooo", "F0000000000000" },

// @LoggerMessage: MCMD1
// @Description: Motor Control Commands - Motors 1-3
// @Field: TimeUS: Time since system startup
// @Field: CmdPos1: Commanded position motor 1
// @Field: CmdPos2: Commanded position motor 2
// @Field: CmdPos3: Commanded position motor 3
// @Field: FbkPos1: Feedback position motor 1
// @Field: FbkPos2: Feedback position motor 2
// @Field: FbkPos3: Feedback position motor 3

    { LOG_MCMD1_MSG, sizeof(log_MotorControlCmd1),
      "MCMD1", "Qffffff", "TimeUS,CmdPos1,CmdPos2,CmdPos3,FbkPos1,FbkPos2,FbkPos3", "sdddddd", "F000000" },

// @LoggerMessage: MCMD2
// @Description: Motor Control Commands - Motors 4-6
// @Field: TimeUS: Time since system startup
// @Field: CmdPos4: Commanded position motor 4
// @Field: CmdPos5: Commanded position motor 5
// @Field: CmdPos6: Commanded position motor 6
// @Field: FbkPos4: Feedback position motor 4
// @Field: FbkPos5: Feedback position motor 5
// @Field: FbkPos6: Feedback position motor 6

    { LOG_MCMD2_MSG, sizeof(log_MotorControlCmd2),
      "MCMD2", "Qffffff", "TimeUS,CmdPos4,CmdPos5,CmdPos6,FbkPos4,FbkPos5,FbkPos6", "sdddddd", "F000000" },
};

void Rover::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else  // LOGGING_ENABLED

// dummy functions
void Rover::Log_Write_Attitude() {}
void Rover::Log_Write_Depth() {}
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Rover::Log_Write_Nav_Tuning() {}
void Rover::Log_Write_Sail() {}
void Rover::Log_Write_Throttle() {}
void Rover::Log_Write_RC(void) {}
void Rover::Log_Write_Steering() {}
void Rover::Log_Write_RobotArm1() {}
void Rover::Log_Write_RobotArm2() {}
void Rover::Log_Write_InterpolatedTrajectory() {}
void Rover::Log_Write_Vehicle_Startup_Messages() {}

#endif  // LOGGING_ENABLED
