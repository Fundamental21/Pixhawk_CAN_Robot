#include <GCS_MAVLink/GCS.h>
#include "AP_Hall_Can_Backend.h"
#include <new>


Hall_Can_Backend* Hall_Can_Backend::_singleton = new (std::nothrow) Hall_Can_Backend();

Hall_Can_Backend::Hall_Can_Backend(){

}

int Hall_Can_Backend::get_index(uint32_t wheel_can_id){
    if (wheel_can_id == Hall_Can_Backend::RIGHT_WHEEL_CAN_ID) {
        return 0;
    }
    if (wheel_can_id == Hall_Can_Backend::LEFT_WHEEL_CAN_ID) {
        return 1;
    } 
    return -1;
}

void Hall_Can_Backend::handle_frame(AP_HAL::CANFrame &frame) {
    uint32_t can_id = get_can_id(frame);
    uint64_t current_time_us = AP_HAL::micros64();
    
    // 直接写入日志消息
    AP::logger().Write_MessageF("CAN RX: ID=0x%02X data[0]=0x%02X", 
                               (unsigned)can_id, (unsigned)frame.data[0]);
    
    // 添加调试信息到GCS
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN RX: ID=0x%02X data[0]=0x%02X", 
                 (unsigned)can_id, (unsigned)frame.data[0]);
    
    // 确定电机索引
    int motor_index = -1;
    switch(can_id) {
        case 0x01: motor_index = 0; break;  // motor 1
        case 0x02: motor_index = 1; break;  // motor 2
        case 0x03: motor_index = 2; break;  // motor 3
        case 0x04: motor_index = 3; break;  // motor 4
        case 0x05: motor_index = 4; break;  // motor 5
        case 0x06: motor_index = 5; break;  // motor 6
        case 0x07: motor_index = 6; break;  // motor 7
        case 0x08: motor_index = 7; break;  // motor 8
        default: 
            // 对于不匹配的CAN ID，也记录一下
            AP::logger().Write_MessageF("CAN: Unknown ID=0x%02X", (unsigned)can_id);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN: Unknown ID=0x%02X", (unsigned)can_id);
            return;
    }
    
    if (motor_index < 0 || motor_index >= 8) return;

    if (frame.data[0] == 0x06) {  // 速度数据
        uint32_t speed_value = (frame.data[1] << 0)  |
                              (frame.data[2] << 8)  |
                              (frame.data[3] << 16) |
                              (frame.data[4] << 24);
        
        motor_states[motor_index].velocity = (float)((speed_value * 0.6f) / 101.0f);
        motor_states[motor_index].last_update_us = current_time_us;
        
        // 写入日志和GCS消息
        AP::logger().Write_MessageF("Motor %d: Speed=%.2f raw=%u", 
                                   motor_index+1, (double)motor_states[motor_index].velocity, 
                                   (unsigned)speed_value);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Motor %d: Speed=%.2f", 
                     motor_index+1, (double)motor_states[motor_index].velocity);
        
    } else if (frame.data[0] == 0x08) {  // 位置数据
        uint32_t pos_value = (frame.data[1] << 0)  |
                            (frame.data[2] << 8)  |
                            (frame.data[3] << 16) |
                            (frame.data[4] << 24);
        
        motor_states[motor_index].position = (float)(((pos_value / (65536.0f * 101.0f)) * 360.0f));
        motor_states[motor_index].last_update_us = current_time_us;
        
        // 写入日志和GCS消息
        AP::logger().Write_MessageF("Motor %d: Pos=%.2f raw=%u", 
                                   motor_index+1, (double)motor_states[motor_index].position,
                                   (unsigned)pos_value);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Motor %d: Pos=%.2f", 
                     motor_index+1, (double)motor_states[motor_index].position);
    }
}

void Hall_Can_Backend::Log_Write_Motor() {
    uint64_t now = AP_HAL::micros64();
    
    // 检查日志记录是否启用
    if (!AP::logger().logging_enabled()) {
        AP::logger().Write_Message("LOG: Motor logging disabled");
        return;
    }
    
    // 记录调用信息
    AP::logger().Write_MessageF("LOG: Log_Write_Motor called at %llu", (unsigned long long)now);
    
    int logged_motors = 0;
    
    // 遍历所有8个电机
    for (int i = 0; i < 8; i++) {
        if (now - motor_states[i].last_update_us < 50000) {  // 50ms内有更新
            struct log_MotorData pkt = {
                LOG_PACKET_HEADER_INIT(LOG_MOTOR_DATA_MSG),
                time_us     : now,
                motor_id    : (uint8_t)(i + 1),
                velocity    : motor_states[i].velocity,
                position    : motor_states[i].position,
                error_code  : motor_states[i].error_code
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));
            logged_motors++;
            
            // 写入详细信息到日志
            AP::logger().Write_MessageF("MOTD: ID=%d V=%.2f P=%.2f age=%llu", 
                                       i+1, (double)motor_states[i].velocity, 
                                       (double)motor_states[i].position,
                                       (unsigned long long)(now - motor_states[i].last_update_us));
            
            // 添加调试信息到GCS
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MOTD: ID=%d V=%.2f P=%.2f", 
                         i+1, (double)motor_states[i].velocity, (double)motor_states[i].position);
        }
    }
    
    // 记录总结信息
    AP::logger().Write_MessageF("LOG: Logged %d motors", logged_motors);
}

void Hall_Can_Backend::Log_Write_CAN_TX(const AP_HAL::CANFrame &frame) {
    // 检查日志记录是否启用
    if (!AP::logger().logging_enabled()) {
        return;
    }
    
    struct log_CANTx pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAN_TX_MSG),
        time_us : AP_HAL::micros64(),
        can_id  : (uint8_t)frame.id,
        dlc     : frame.dlc,
        data    : {frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                   frame.data[4], frame.data[5], frame.data[6], frame.data[7]}
    };
    
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
    
    // 同时写入文本消息便于调试
    AP::logger().Write_MessageF("CAN TX LOG: ID=0x%02X DLC=%d", 
                               (unsigned)frame.id, (unsigned)frame.dlc);
}

bool Hall_Can_Backend::is_query_ws_frame(AP_HAL::CANFrame &frame){
    uint8_t ft = frame.data[0];
    uint8_t qt = frame.data[1];
    if (ft == 0x0F && qt == 0x01) {
        return true;
    }
    return false;
}

bool Hall_Can_Backend::is_error_frame(AP_HAL::CANFrame &frame){
    uint8_t ft = frame.data[0];
    uint8_t qt = frame.data[1];
    if (ft == 0x0F && qt == 0x00) {
        return true;
    }
    return false;
}

bool Hall_Can_Backend::is_speed_frame(AP_HAL::CANFrame &frame){
    uint8_t ft = frame.data[0];
    uint8_t qt = frame.data[1];
    if (ft == 0x0F && qt == 0x01) {
        return true;
    }
    return false;
}

bool Hall_Can_Backend::is_voltage_frame(AP_HAL::CANFrame &frame){
    uint8_t ft = frame.data[0];
    uint8_t qt = frame.data[1];
    if (ft == 0x0F && qt == 0x02) {
        return true;
    }
    return false;
}

bool Hall_Can_Backend::is_mos_tmp_frame(AP_HAL::CANFrame &frame){
    uint8_t ft = frame.data[0];
    uint8_t qt = frame.data[1];
    if (ft == 0x0F && qt == 0x03) {
        return true;
    }
    return false;
}


bool Hall_Can_Backend::is_wheel_hall_id(uint32_t can_id){
    if (can_id != Hall_Can_Backend::RIGHT_WHEEL_CAN_ID && can_id != Hall_Can_Backend::LEFT_WHEEL_CAN_ID) {
        return false;
    }
    return true;
}

int32_t Hall_Can_Backend::get_can_id(AP_HAL::CANFrame &frame){
    return frame.id & AP_HAL::CANFrame::MaskStdID;
}

int32_t Hall_Can_Backend::parse_data_int32(AP_HAL::CANFrame &frame){
    return (frame.data[2] << 24) | (frame.data[3] << 16) | (frame.data[4] << 8) | (frame.data[5]);
}

Hall_Can_Backend::Hall_Can_Data* Hall_Can_Backend::get_data(uint32_t wheel_can_id){
    if (!is_wheel_hall_id(wheel_can_id)) {
        return nullptr;
    }

    return &this->data[get_index(wheel_can_id)];
}

int32_t Hall_Can_Backend::get_wheel_speed(uint32_t wheel_can_id){
    if (!is_wheel_hall_id(wheel_can_id)) {
        return 0;
    }
    int wheel_index = get_index(wheel_can_id);

    if (AP_HAL::micros64() - this->data[wheel_index].ts  > 500 * 1000) {
        return 0;
    }

    return this->data[wheel_index].wheel_speed;
}