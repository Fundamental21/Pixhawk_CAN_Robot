#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/RingBuffer.h>

// Forward declaration for AP_RobotArm CAN processing
namespace RobotArm {
    class AP_CAN_Processor;
}

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

    // CAN帧队列项结构 - 与AP_CAN_Processor::CAN_Frame_Item保持一致
    struct CAN_Frame_Item {
        AP_HAL::CANFrame frame;
        uint64_t timestamp_us;
        uint8_t can_bus_id;  // CAN总线ID: 1=CAN1, 2=CAN2
        
        CAN_Frame_Item() : timestamp_us(0), can_bus_id(0) {}
        CAN_Frame_Item(const AP_HAL::CANFrame& f, uint64_t ts, uint8_t bus_id) 
            : frame(f), timestamp_us(ts), can_bus_id(bus_id) {}
    };

    static Hall_Can_Backend *get_singleton(void) { return _singleton; }
    static Hall_Can_Backend *_singleton;

    void handle_frame(AP_HAL::CANFrame &frame);
    void handle_frame(AP_HAL::CANFrame &frame, uint8_t can_bus_id);  // 新增：支持CAN总线区分的版本
    void Log_Write_Motor();  // 日志写入方法
    void Log_Write_CAN_TX(const AP_HAL::CANFrame &frame);  // CAN发送日志记录方法

    int32_t get_wheel_speed(uint32_t wheel_can_id);
    Hall_Can_Data* get_data(uint32_t wheel_can_id);
    
    // 获取机械臂CAN帧队列的访问接口
    ObjectBuffer<CAN_Frame_Item>* get_robotarm_can_queue(void) { return robotarm_can_queue; }
    
    // 设置机械臂CAN处理器（由AP_RobotArm调用）
    void set_robotarm_processor(RobotArm::AP_CAN_Processor* processor) { robotarm_processor = processor; }

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
    
    // 机械臂CAN帧队列 - 线程安全的环形缓冲区
    ObjectBuffer<CAN_Frame_Item>* robotarm_can_queue;
    
    // 机械臂CAN处理器指针
    RobotArm::AP_CAN_Processor* robotarm_processor;
    
    // 队列统计信息
    struct {
        uint32_t frames_queued;
        uint32_t queue_overflows;
        uint32_t frames_processed;
    } queue_stats;
};
