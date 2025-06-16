#include "CAN_Robot_Rx_Queue.h"
#include <new>

// 单例实例
CAN_Robot_Rx_Queue* CAN_Robot_Rx_Queue::_singleton = nullptr;

// 初始化静态实例
void CAN_Robot_Rx_Queue::init(void)
{
    if (_singleton == nullptr) {
        _singleton = new (std::nothrow) CAN_Robot_Rx_Queue();
    }
}

CAN_Robot_Rx_Queue::CAN_Robot_Rx_Queue() :
    _write_count(0),
    _read_count(0),
    _total_received(0),
    _total_dropped(0),
    _can1_count(0),
    _can2_count(0)
{
    // 初始化队列
    for (int i = 0; i < CAN_RX_QUEUE_SIZE; i++) {
        _queue[i] = CANRxMessage();
    }
}

bool CAN_Robot_Rx_Queue::push_message(const CANRxMessage& msg)
{
    WITH_SEMAPHORE(_sem);
    
    // 检查队列是否满
    if (is_queue_full()) {
        // 队列满时，自动删除最前面的消息（推进读指针）
        _read_count++;
        _total_dropped++;  // 统计被删除的消息
    }
    
    // 添加消息到队列末尾
    uint32_t index = _write_count % CAN_RX_QUEUE_SIZE;
    _queue[index] = msg;
    _write_count++;
    
    // 更新统计
    _total_received++;
    if (msg.can_channel == 0) {
        _can1_count++;
    } else if (msg.can_channel == 1) {
        _can2_count++;
    }
    
    return true;
}

bool CAN_Robot_Rx_Queue::get_next_message(CANRxMessage& msg)
{
    WITH_SEMAPHORE(_sem);
    
    // 检查队列是否空
    if (is_queue_empty()) {
        return false;
    }
    
    // 从队列获取消息
    uint32_t index = _read_count % CAN_RX_QUEUE_SIZE;
    msg = _queue[index];
    
    return true;
}

void CAN_Robot_Rx_Queue::mark_message_processed(void)
{
    WITH_SEMAPHORE(_sem);
    
    if (!is_queue_empty()) {
        _read_count++;
    }
}

void CAN_Robot_Rx_Queue::reset_stats(void)
{
    WITH_SEMAPHORE(_sem);
    
    _total_received = 0;
    _total_dropped = 0;
    _can1_count = 0;
    _can2_count = 0;
} 