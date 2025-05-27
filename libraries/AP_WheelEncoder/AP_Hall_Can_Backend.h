#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

class Hall_Can_Backend {
public:
    static const uint32_t RIGHT_WHEEL_CAN_ID = 30;
    static const uint32_t LEFT_WHEEL_CAN_ID = 29;

    Hall_Can_Backend();
    CLASS_NO_COPY(Hall_Can_Backend);

    struct Hall_Can_Data {
        uint32_t can_id;
        uint64_t ts;
        AP_HAL::CANFrame *frame;
        int32_t wheel_speed;
    };

    static Hall_Can_Backend *get_singleton(void) { return _singleton; }
    static Hall_Can_Backend *_singleton;

    void handle_frame(AP_HAL::CANFrame &frame);
    void Log_Write_Motor();  // 日志写入方法
    void Log_Write_CAN_TX(const AP_HAL::CANFrame &frame);  // CAN发送日志记录方法

    int32_t get_wheel_speed(uint32_t wheel_can_id);
    Hall_Can_Data* get_data(uint32_t wheel_can_id);

private:
    int32_t get_can_id(AP_HAL::CANFrame &frame);
    int32_t parse_data_int32(AP_HAL::CANFrame &frame);
    bool is_wheel_hall_id(uint32_t can_id);
    int get_index(uint32_t wheel_can_id);
    bool is_query_ws_frame(AP_HAL::CANFrame &frame);

    bool is_error_frame(AP_HAL::CANFrame &frame);
    bool is_speed_frame(AP_HAL::CANFrame &frame);
    bool is_voltage_frame(AP_HAL::CANFrame &frame);
    bool is_mos_tmp_frame(AP_HAL::CANFrame &frame);
    
    // 电机状态结构
    struct MotorState {
        float velocity;
        float position;
        uint16_t error_code;
        uint64_t last_update_us;
        
        MotorState() : velocity(0), position(0), error_code(0), last_update_us(0) {}
    };
    
    Hall_Can_Data data[2];
    MotorState motor_states[8];  // 支持8个电机，索引0-7
};
