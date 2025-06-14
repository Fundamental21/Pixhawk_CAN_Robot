#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define CAN_RX_QUEUE_SIZE 100  // 队列大小

// CAN接收消息队列类
class CAN_Robot_Rx_Queue {
public:
    // CAN接收消息结构
    struct CANRxMessage {
        uint32_t can_id;        // CAN ID
        uint8_t can_channel;    // CAN通道 (0=CAN1, 1=CAN2)
        uint8_t dlc;           // 数据长度
        uint8_t data[8];       // 数据内容
        uint64_t timestamp_us; // 接收时间戳(微秒)
        
        CANRxMessage() {
            can_id = 0;
            can_channel = 0;
            dlc = 0;
            memset(data, 0, sizeof(data));
            timestamp_us = 0;
        }
    };

    // 单例模式
    static void init(void);
    static CAN_Robot_Rx_Queue* get_singleton(void) { return _singleton; }
    
    // 队列操作
    bool push_message(const CANRxMessage& msg);
    bool get_next_message(CANRxMessage& msg);
    void mark_message_processed(void);
    
    // 队列状态
    uint32_t get_queue_size(void) const { return _write_count - _read_count; }
    bool is_queue_full(void) const { return get_queue_size() >= CAN_RX_QUEUE_SIZE; }
    bool is_queue_empty(void) const { return _write_count == _read_count; }
    
    // 统计信息
    uint32_t get_total_received(void) const { return _total_received; }
    uint32_t get_total_dropped(void) const { return _total_dropped; }
    uint32_t get_can1_count(void) const { return _can1_count; }
    uint32_t get_can2_count(void) const { return _can2_count; }
    
    // 重置统计
    void reset_stats(void);

private:
    static CAN_Robot_Rx_Queue *_singleton;
    
    // 队列数据
    CANRxMessage _queue[CAN_RX_QUEUE_SIZE];
    volatile uint32_t _write_count;  // 写入计数
    volatile uint32_t _read_count;   // 读取计数
    
    // 统计信息
    uint32_t _total_received;
    uint32_t _total_dropped;
    uint32_t _can1_count;
    uint32_t _can2_count;
    
    // 信号量保护
    HAL_Semaphore _sem;
    
    // 防止直接构造
    CAN_Robot_Rx_Queue();
}; 