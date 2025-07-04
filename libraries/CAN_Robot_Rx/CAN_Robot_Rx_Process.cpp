#include "CAN_Robot_Rx_Process.h"
#include <new>
#include <cstring>  // for memcpy
#include <GCS_MAVLink/GCS.h>

// 单例实例
CAN_Robot_Rx_Process* CAN_Robot_Rx_Process::_singleton = nullptr;

// 初始化静态实例
void CAN_Robot_Rx_Process::init(void)
{
    if (_singleton == nullptr) {
        _singleton = new (std::nothrow) CAN_Robot_Rx_Process();
    }
}

// 多帧缓存结构体 - 支持双臂独立控制
struct CAN2TrajectoryFrameCache {
    float joints[6];    // 6个关节位置数据
    bool received[6];   // 6个关节接收标志
    bool is_final;
    uint64_t timestamp_us;
    CAN2TrajectoryFrameCache() : is_final(false), timestamp_us(0) {
        for (int i = 0; i < 6; i++) joints[i] = 0;
        for (int i = 0; i < 6; i++) received[i] = false;
    }
};
// 为每条机械臂分别缓存轨迹点（最多16个轨迹点）
static CAN2TrajectoryFrameCache traj_cache[2][16]; // [arm_id][point_id]

CAN_Robot_Rx_Process::CAN_Robot_Rx_Process() :
    _mit_msg_count(0),
    _kegu_msg_count(0),
    _cmd_msg_count(0),
    _last_log_ms(0)
{
    // 初始化电机状态数组
    memset(_mit_motors, 0, sizeof(_mit_motors));
    memset(_kegu_motors, 0, sizeof(_kegu_motors));
}

void CAN_Robot_Rx_Process::process_all_rx_messages(void)
{
    CAN_Robot_Rx_Queue* rx_queue = CAN_Robot_Rx_Queue::get_singleton();
    if (!rx_queue) {
        return;
    }
    
    CAN_Robot_Rx_Queue::CANRxMessage msg;
    uint32_t processed_count = 0;
    for (uint8_t i = 0; i < 3; i++) {
    while (rx_queue->get_next_message(msg)) {
        // 根据Motor ID类型分发消息
        if (is_mit_motor_id(msg.motor_id)) {
            process_mit_motor_message(msg);
        } else if (is_kegu_motor_id(msg.motor_id)) {
            process_kegu_motor_message(msg);
        } else {
            // 处理其他机器人通信消息
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ROBOT_MSG_RECEIVED");
            process_robot_command_message(msg);
        }
        
        // 标记消息已处理
        rx_queue->mark_message_processed();
        processed_count++;
        
        // 防止单次处理过多消息影响实时性
        if (processed_count >= 20) {
            break;
        }
    }
    // 定期记录状态
    log_motor_status();
    }
    
    
}

void CAN_Robot_Rx_Process::process_mit_motor_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    if (msg.can_id > 1) return; // 只支持CAN1和CAN2
    
    uint8_t motor_id = extract_motor_id(msg.motor_id);
    if (motor_id >= MAX_MOTORS_PER_CAN) return;
    
    uint8_t can_channel = msg.can_id; // CAN1=0, CAN2=1
    MIT_Motor_Feedback &feedback = _mit_motors[can_channel][motor_id];
    decode_mit_feedback(msg, feedback);
    feedback.last_update_us = msg.timestamp_us;
    
    _mit_msg_count++;
    
    // 记录详细日志
    AP::logger().Write_MessageF("MIT_RX: CAN%d ID:0x%X M%d Pos:%.2f Vel:%.2f Curr:%.2f Temp:%.1f", 
                               (int)msg.can_id + 1, (unsigned)msg.motor_id, motor_id,
                               feedback.position, feedback.velocity, 
                               feedback.current, feedback.temperature);
}

void CAN_Robot_Rx_Process::process_kegu_motor_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    if (msg.can_id > 1) return; // 只支持CAN1和CAN2
    
    uint8_t motor_index = extract_motor_id(msg.motor_id);  // 这现在是数组索引
    if (motor_index >= MAX_MOTORS_PER_CAN) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "KEGU Motor Index Out of Range: CAN_ID=0x%lX index=%d max=%d", 
        //              (unsigned long)msg.motor_id, motor_index, MAX_MOTORS_PER_CAN);
        return;
    }
    
    // 确定是哪个夹爪 - 支持两个夹爪：索引6(ARM1)和索引13(ARM2)
    KEGU_Motor_Feedback *feedback_ptr = nullptr;
    uint8_t gripper_id = 0;  // 0=ARM1, 1=ARM2
    
    if (motor_index == 6) {
        // ARM1夹爪：motor_id=7, 数组索引=6
        feedback_ptr = &_kegu_motors[0][motor_index];
        gripper_id = 0;
    } else if (motor_index == 13) {
        // ARM2夹爪：motor_id=14, 数组索引=13  
        feedback_ptr = &_kegu_motors[0][motor_index];
        gripper_id = 1;
    } else {
        // 不是夹爪电机，忽略
        // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "KEGU: Ignoring non-gripper motor index %d", motor_index);
        return;
    }
    
    // 添加调试信息：显示接收到的原始消息
    uint8_t actual_motor_id = (msg.motor_id == 0x07) ? 7 : 
                              (msg.motor_id == 0x0E) ? 14 : 
                              (motor_index + 1);
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "KEGU_MSG_RX: CAN%d CAN_ID=0x%lX Motor_ID=%d index=%d DLC=%d ARM=%d", 
    //              (int)msg.can_id + 1, (unsigned long)msg.motor_id, actual_motor_id, motor_index, msg.dlc, gripper_id+1);
    
    decode_kegu_feedback(msg, *feedback_ptr);
    feedback_ptr->last_update_us = msg.timestamp_us;
    
    _kegu_msg_count++;
    
    // 记录详细日志（明确显示CAN ID和电机ID的区别，以及是哪个夹爪）
    AP::logger().Write_MessageF("ARM%d_KEGU_RX: CAN%d CAN_ID:0x%X Motor_ID:%d(idx%d) Spd:%.1f Curr:%.1f Pos:%d", 
                               gripper_id+1, (int)msg.can_id + 1, (unsigned)msg.motor_id, actual_motor_id, motor_index,
                               feedback_ptr->speed, feedback_ptr->current, (int)feedback_ptr->position);
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ARM%d_KEGU_RX: CAN%d CAN_ID:0x%X Motor_ID:%d(idx%d) Spd:%.1f Curr:%.1f Pos:%d", 
    //                            gripper_id+1, (int)msg.can_id + 1, (unsigned)msg.motor_id, actual_motor_id, motor_index,
    //                            feedback_ptr->speed, feedback_ptr->current, (int)feedback_ptr->position);

}

void CAN_Robot_Rx_Process::process_robot_command_message(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    _cmd_msg_count++;
    
    // 添加调试信息：显示接收到的所有机器人命令消息的详细信息
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ROBOT_CMD_DBG: CAN%d ID:0x%X DLC:%d", 
                  (int)msg.can_id + 1, (unsigned)msg.motor_id, msg.dlc);
    
    // 检查是否为CAN2轨迹命令 (CAN2, ID = 0x100)
    if (msg.can_id == 1 && msg.motor_id == 0x100 && msg.dlc >= 8) {
        // 处理CAN2轨迹数据
        process_can2_trajectory_command(msg);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN2_TRAJ_RX");
        return;
    } else if (msg.can_id == 1 && msg.motor_id == 0x100) {
        // CAN2轨迹命令但数据长度不足
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAN2_TRAJ_SHORT: DLC=%d (need >=8)", msg.dlc);
        return;
    }
    
    // 检查是否为KEGU电机控制命令 (CAN2, ID = 0x150 ARM1夹爪 或 0x160 ARM2夹爪)
    if (msg.can_id == 1 && (msg.motor_id == 0x150 || msg.motor_id == 0x160) && msg.dlc >= 1) {
        // 确定夹爪ID
        uint8_t gripper_id = (msg.motor_id == 0x150) ? 0 : 1;  // 0x150=ARM1, 0x160=ARM2
        
        // 处理KEGU电机控制命令
        process_kegu_control_command(msg, gripper_id);
        return;
    }
    
    // 处理其他机器人指令消息
    // 这里可以根据具体的机器人通信协议来实现
    AP::logger().Write_MessageF("ROBOT_CMD: CAN%d ID:0x%X DLC:%d [%02X %02X %02X %02X]", 
                               (int)msg.can_id + 1, (unsigned)msg.motor_id, msg.dlc,
                               (unsigned)msg.data[0], (unsigned)msg.data[1], 
                               (unsigned)msg.data[2], (unsigned)msg.data[3]);
}

// 新增：处理CAN2轨迹命令
void CAN_Robot_Rx_Process::process_can2_trajectory_command(const CAN_Robot_Rx_Queue::CANRxMessage &msg)
{
    if (msg.dlc < 8) return; // 数据长度不足

    uint8_t point_id = msg.data[0]; // 位置点编号（00, 01, 02...）
    uint8_t can2_motor_id = msg.data[1]; // CAN2协议电机编号（00-11对应12个电机：ARM1[0-5], ARM2[6-11]）

    // 检查CAN2协议电机ID是否有效（支持双臂12个电机）
    if (can2_motor_id >= 12) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAN2_TRAJ: Invalid CAN2 motor_id %d (max 11)", can2_motor_id);
        return;
    }
    
    // 确定机械臂ID和关节ID
    uint8_t arm_id = (can2_motor_id < 6) ? 0 : 1;  // ARM1: 0-5, ARM2: 6-11
    uint8_t joint_id = can2_motor_id % 6;          // 关节ID: 0-5

    // CAN2协议motor_id转换为实际MIT电机motor_id
    // CAN2协议: ARM1[0-5] -> MIT电机[1-6], ARM2[6-11] -> MIT电机[8-13]
    uint8_t actual_motor_id;
    if (arm_id == 0) {
        actual_motor_id = can2_motor_id + 1;  // ARM1: 0->1, 1->2, ..., 5->6
    } else {
        actual_motor_id = can2_motor_id + 2;  // ARM2: 6->8, 7->9, ..., 11->13
    }

    // 解析4字节位置数据（直接接收float格式）
    union {
        float f;
        uint8_t bytes[4];
    } position_union;
    
    // 从CAN数据中读取4字节位置数据（小端格式）
    position_union.bytes[0] = msg.data[2]; // Position byte 0 (LSB)
    position_union.bytes[1] = msg.data[3]; // Position byte 1
    position_union.bytes[2] = msg.data[4]; // Position byte 2
    position_union.bytes[3] = msg.data[5]; // Position byte 3 (MSB)
    
    float position_value = position_union.f;
    
    // 发送单个电机位置值到地面站，包含机械臂信息和实际motor_id
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN2_POS: Point%d ARM%d_J%d(MIT_ID%d) Pos:%.2f Final:%d", 
                  point_id, arm_id+1, joint_id, actual_motor_id, position_value, (msg.data[7] == 0x01) ? 1 : 0);
    
    // 检查是否为最终点（第8字节是01表示最后一个点的最后一帧）
    bool is_final_frame = (msg.data[7] == 0x01);

    // 添加调试信息显示接收到的原始数据
    #ifdef ARDUPILOT_BUILD
    extern const AP_HAL::HAL& hal;
    hal.console->printf("CAN2_TRAJ_RX: Point=%d CAN2_ID=%d MIT_ID=%d Position=%.2f Final=%d [%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                       point_id, can2_motor_id, actual_motor_id, position_value, is_final_frame ? 1 : 0,
                       msg.data[0], msg.data[1], msg.data[2], msg.data[3], 
                       msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    #endif

    // 写入对应机械臂的缓存（直接存储float值）
    traj_cache[arm_id][point_id].joints[joint_id] = position_value;
    traj_cache[arm_id][point_id].received[joint_id] = true;
    
    // 如果这是最终帧，标记整个轨迹点为最终点
    if (is_final_frame) {
        traj_cache[arm_id][point_id].is_final = true;
    }
    
    traj_cache[arm_id][point_id].timestamp_us = msg.timestamp_us;

    // 检查这条机械臂的6个关节数据是否都收齐了
    bool all_joints_received = true;
    for (uint8_t i = 0; i < 6; i++) {
        if (!traj_cache[arm_id][point_id].received[i]) {
            all_joints_received = false;
            break;
        }
    }

    // 如果这条机械臂的6个关节数据都收齐了
    if (all_joints_received) {
        CAN2TrajectoryData trajectory_item;
        trajectory_item.arm_id = arm_id;  // 设置机械臂ID
        for (int i = 0; i < 6; ++i) {
            trajectory_item.joint_positions[i] = traj_cache[arm_id][point_id].joints[i]; // 直接使用float值
        }
        trajectory_item.is_final_point = traj_cache[arm_id][point_id].is_final;
        trajectory_item.timestamp_us = traj_cache[arm_id][point_id].timestamp_us;
        
        // 推送到对应机械臂的队列
        extern TrajectoryDataQueue trajectory_queues[2];
        if (trajectory_queues[arm_id].push(trajectory_item)) {
            
            // 发送轨迹数据到地面站，明确标识机械臂
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN2_TRAJ: ARM%d Point%d [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] Final:%d Queue:%d",
                arm_id+1, point_id,
                trajectory_item.joint_positions[0], trajectory_item.joint_positions[1],
                trajectory_item.joint_positions[2], trajectory_item.joint_positions[3],
                trajectory_item.joint_positions[4], trajectory_item.joint_positions[5],
                trajectory_item.is_final_point ? 1 : 0, trajectory_queues[arm_id].size());
            
            #ifdef ARDUPILOT_BUILD
            extern const AP_HAL::HAL& hal;
            hal.console->printf("CAN2 Trajectory ARM%d Point%d: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] Final:%d Queue:%d\n",
                arm_id+1, point_id,
                trajectory_item.joint_positions[0], trajectory_item.joint_positions[1],
                trajectory_item.joint_positions[2], trajectory_item.joint_positions[3],
                trajectory_item.joint_positions[4], trajectory_item.joint_positions[5],
                trajectory_item.is_final_point ? 1 : 0, trajectory_queues[arm_id].size());
            #endif
        } else {
            AP::logger().Write_MessageF("CAN2_TRAJ_WARNING: ARM%d Queue full, dropping trajectory point %d", arm_id+1, point_id);
            #ifdef ARDUPILOT_BUILD
            extern const AP_HAL::HAL& hal;
            hal.console->printf("CAN2 Trajectory WARNING: ARM%d Queue full, dropping trajectory point %d\n", arm_id+1, point_id);
            #endif
        }
        
        // 清空对应机械臂的缓存，准备接收下一个轨迹点
        for (int i = 0; i < 6; i++) {
            traj_cache[arm_id][point_id].received[i] = false;
        }
        traj_cache[arm_id][point_id].is_final = false;
    }
    
    last_trajectory_receive_time = AP_HAL::millis();
}

// 新增：处理KEGU电机控制命令
void CAN_Robot_Rx_Process::process_kegu_control_command(const CAN_Robot_Rx_Queue::CANRxMessage &msg, uint8_t gripper_id)
{
    if (msg.dlc < 1) return; // 数据长度不足
    if (gripper_id >= 2) return; // 无效的夹爪ID
    
    uint8_t command = msg.data[0];
    
    // 记录接收到的命令
    AP::logger().Write_MessageF("ARM%d_KEGU_CMD: CAN%d ID:0x%X CMD:0x%02X DLC:%d", 
                               gripper_id+1, (int)msg.can_id + 1, (unsigned)msg.motor_id, command, msg.dlc);
    
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ARM%d KEGU Command received: 0x%02X", gripper_id+1, command);
    
    // 设置双夹爪全局标志，让robot_arm_control_loop处理
    extern uint8_t kegu_external_commands[2];
    extern bool kegu_external_commands_received[2];
    extern uint32_t kegu_command_timestamps[2];
    
    kegu_external_commands[gripper_id] = command;
    kegu_external_commands_received[gripper_id] = true;
    kegu_command_timestamps[gripper_id] = AP_HAL::millis();
    
    // 向后兼容：如果是ARM1夹爪，同时设置原有的全局变量
    if (gripper_id == 0) {
        extern uint8_t kegu_external_command;
        extern bool kegu_external_command_received;
        extern uint32_t kegu_command_timestamp;
        
        kegu_external_command = command;
        kegu_external_command_received = true;
        kegu_command_timestamp = AP_HAL::millis();
    }
    
    #ifdef ARDUPILOT_BUILD
    extern const AP_HAL::HAL& hal;
    hal.console->printf("ARM%d KEGU Control Command: 0x%02X received\n", gripper_id+1, command);
    #endif
}

bool CAN_Robot_Rx_Process::is_mit_motor_id(uint32_t motor_id) const
{
    // MIT电机使用Motor ID:
    // ARM1: 0x01-0x06 (1-6) 
    // ARM2: 0x08-0x0D (8-13)
    return ((motor_id >= 0x01 && motor_id <= 0x06) || 
            (motor_id >= 0x08 && motor_id <= 0x0D));
}

bool CAN_Robot_Rx_Process::is_kegu_motor_id(uint32_t motor_id) const
{
    // KEGU电机使用特定的ID模式
    uint16_t high_part = (motor_id >> 8) & 0xFF;
    // 夹爪电机ID=7和ID=14都是KEGU类型，用于自动上报数据
    return (high_part == 0x02 || high_part == 0x03);
}

uint8_t CAN_Robot_Rx_Process::extract_motor_id(uint32_t motor_id) const
{
    if (is_kegu_motor_id(motor_id)) {
        // KEGU电机：从CAN ID中提取电机ID
        uint16_t high_part = (motor_id >> 8) & 0xFF;
        if (high_part == 0x02 || high_part == 0x03) {
            uint8_t extracted_id = motor_id & 0x0F;  // CAN ID格式：0x2XY或0x3XY，电机ID在低4位
            // motor_id = 0x287 or 0x28E, extracted_id = 0x07 or 0x0E
            // 验证提取的ID是否在有效范围内（1-14为有效电机ID）
            if (extracted_id >= 1 && extracted_id <= 14) {
                return extracted_id - 1;  // 转换为0-based数组索引（0-13）
            } else {
                // 无效的电机ID，记录错误并返回默认值
                // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "KEGU Invalid Motor ID: 0x%lX extracted_id=%d", 
                //              (unsigned long)motor_id, extracted_id);
                return 0;  // 返回默认索引0
            }
        } 
    }
    // MIT电机：motor_id就是电机ID，需要转换为0-based索引
    uint8_t extracted_id = motor_id & 0xFF;
    return (extracted_id > 0) ? (extracted_id - 1) : 0;
}

void CAN_Robot_Rx_Process::decode_mit_feedback(const CAN_Robot_Rx_Queue::CANRxMessage &msg, MIT_Motor_Feedback &feedback)
{
    if (msg.dlc < 5) return; // 数据长度不足
    
    uint8_t cmd = msg.data[0];
    
    // 从数据字节1-4提取32位值（小端格式）
    int32_t raw_value = (msg.data[1] << 0) | (msg.data[2] << 8) | 
                        (msg.data[3] << 16) | (msg.data[4] << 24);
    
    switch (cmd) {
        case 0x06: // 速度反馈
            feedback.velocity = raw_value * 0.6f / 101.0f; // 0.01 RPM 分辨率
            break;
        case 0x08: // 位置反馈
            feedback.position = raw_value * 360.0f / 262144.0f; // 0.01 度分辨率
            break;
        case 0x04: // 电流反馈
            feedback.current = raw_value * 1.0f; // mA 分辨率，直接使用mA单位
            break;
        case 0x32: // 温度反馈
            feedback.temperature = static_cast<float>(raw_value);
            break;
        default:
            // 未知命令，记录错误
            feedback.error_code = cmd;
            break;
    }
}

void CAN_Robot_Rx_Process::decode_kegu_feedback(const CAN_Robot_Rx_Queue::CANRxMessage &msg, KEGU_Motor_Feedback &feedback)
{
    // 使用实际CAN ID而不是CAN总线ID来判断消息类型
    uint16_t msg_type = (msg.motor_id >> 8) & 0xFF;
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "KEGU CAN_ID: 0x%lX, msg_type: 0x%02X", (unsigned long)msg.motor_id, msg_type);
    
    if (msg_type == 0x02 && msg.dlc >= 6) {
        // 0x280+电机ID: 速度和电流反馈 (6字节)
        // byte0-3: 当前速度(RPM), byte4-5: 当前电流(10mA单位)
        int32_t speed_raw;
        memcpy(&speed_raw, msg.data, sizeof(int32_t));
        feedback.speed = speed_raw; // 直接使用RPM值
        
        int16_t current_raw;
        memcpy(&current_raw, msg.data + 4, sizeof(int16_t));
        feedback.current = current_raw * 10.0f; // 10mA单位转换为mA单位 (10mA * 10 = mA)
        
        // 记录详细的电流解码信息
        AP::logger().Write_MessageF("KEGU_CURRENTrx: CAN_ID:0x%X Raw:%d(10mA) Decoded:%.1f(mA)", 
                                   (unsigned)msg.motor_id, current_raw, feedback.current);
                // 同时使用GCS发送文本消息
        // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "KEGU Currentrx: %.2fmA", feedback.current);
    } else if (msg_type == 0x03 && msg.dlc >= 4) {
        // 0x380+电机ID: 位置反馈 (4字节)
        // byte0-3: 当前位置
        int32_t position_raw;
        memcpy(&position_raw, msg.data, sizeof(int32_t));
        feedback.position = position_raw; // 直接使用原始位置值,脉冲数
    }
    
}

const CAN_Robot_Rx_Process::MIT_Motor_Feedback& CAN_Robot_Rx_Process::get_mit_motor_status(uint8_t can_channel, uint8_t motor_id) const
{
    static MIT_Motor_Feedback empty_feedback;
    
    if (can_channel > 1 || motor_id >= MAX_MOTORS_PER_CAN) {
        return empty_feedback;
    }
    
    return _mit_motors[can_channel][motor_id];
}

const CAN_Robot_Rx_Process::KEGU_Motor_Feedback& CAN_Robot_Rx_Process::get_kegu_motor_status(uint8_t can_channel, uint8_t motor_id) const
{
    static KEGU_Motor_Feedback empty_feedback;
    
    if (can_channel > 1 || motor_id >= MAX_MOTORS_PER_CAN) {
        return empty_feedback;
    }
    
    return _kegu_motors[can_channel][motor_id];
}

void CAN_Robot_Rx_Process::log_motor_status(void)
{
    uint32_t now_ms = AP_HAL::millis();
    
    // 每10秒记录一次状态总结
    if (now_ms - _last_log_ms > 10000) {
        _last_log_ms = now_ms;
        
        AP::logger().Write_MessageF("CAN_ROBOT_STATUS: MIT:%u KEGU:%u CMD:%u", 
                                   (unsigned)_mit_msg_count, 
                                   (unsigned)_kegu_msg_count, 
                                   (unsigned)_cmd_msg_count);
        
        // 记录活跃电机状态
        for (uint8_t can = 0; can < 2; can++) {
            for (uint8_t motor = 0; motor < MAX_MOTORS_PER_CAN; motor++) {
                const MIT_Motor_Feedback &mit = _mit_motors[can][motor];
                if (mit.last_update_us > 0 && (AP_HAL::micros64() - mit.last_update_us) < 1000000) {
                    AP::logger().Write_MessageF("MIT_STATUS: CAN%d M%d Pos:%.2f Vel:%.2f", 
                                               can + 1, motor, mit.position, mit.velocity);
                }
                
                const KEGU_Motor_Feedback &kegu = _kegu_motors[can][motor];
                if (kegu.last_update_us > 0 && (AP_HAL::micros64() - kegu.last_update_us) < 1000000) {
                    AP::logger().Write_MessageF("KEGU_STATUS: CAN%d M%d Spd:%.1f Pos:%d", 
                                               can + 1, motor, kegu.speed, kegu.position);
                }
            }
        }
    }
}

// 实现轨迹数据队列方法
bool TrajectoryDataQueue::push(const CAN2TrajectoryData& item)
{
    if (is_full()) {
        return false;  // 队列已满
    }
    
    data[tail] = item;
    tail = (tail + 1) % MAX_QUEUE_SIZE;
    count++;
    return true;
}

bool TrajectoryDataQueue::pop(CAN2TrajectoryData& item)
{
    if (is_empty()) {
        return false;  // 队列为空
    }
    
    item = data[head];
    head = (head + 1) % MAX_QUEUE_SIZE;
    count--;
    return true;
} 