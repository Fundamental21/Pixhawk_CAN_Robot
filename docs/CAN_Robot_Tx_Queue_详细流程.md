# CAN Robot Tx 消息队列详细流程分析

## 1. 队列的建立 (Queue Creation)

### 1.1 单例模式初始化
```cpp
// 在 CAN_Robot_Tx_Queue.cpp 中
CAN_Robot_Tx_Queue* CAN_Robot_Tx_Queue::_singleton = new (std::nothrow) CAN_Robot_Tx_Queue();
```

### 1.2 队列结构定义
```cpp
// 在 CAN_Robot_Tx_Queue.h 中
class CAN_Robot_Tx_Queue {
private:
    static const uint32_t QUEUE_SIZE = 100;  // 队列大小
    
    // 核心队列数据结构
    MotorCommand _command_queue[QUEUE_SIZE];  // 消息队列本体（数组）
    uint32_t _queue_head;                     // 队列头指针（消费端）
    uint32_t _queue_tail;                     // 队列尾指针（生产端）
    uint32_t _queue_count;                    // 当前队列中的消息数量
    
    // 统计信息
    uint32_t _commands_sent;                  // 已发送命令数
    uint32_t _commands_dropped;               // 已丢弃命令数
    
    // 线程安全
    HAL_Semaphore _queue_semaphore;           // 信号量保护
};
```

### 1.3 队列初始化
```cpp
CAN_Robot_Tx_Queue::CAN_Robot_Tx_Queue() :
    _queue_head(0),        // 头指针初始化为0
    _queue_tail(0),        // 尾指针初始化为0
    _queue_count(0),       // 消息数量初始化为0
    _commands_sent(0),
    _commands_dropped(0),
    _last_log_ms(0)
{
    // 初始化所有命令为已处理状态
    for (uint32_t i = 0; i < QUEUE_SIZE; i++) {
        _command_queue[i].processed = true;
    }
}
```

## 2. Push消息进队列 (Message Queuing)

### 2.1 消息入队调用链
```
MIT_Motor.cpp (MotorControl_Handler) 
    ↓
queue->queue_motor_command()
    ↓
CAN_Robot_Tx_Queue::queue_motor_command()
```

### 2.2 具体Push过程
```cpp
void CAN_Robot_Tx_Queue::queue_motor_command(uint8_t can_id, uint8_t motor_id, 
                                            MotorType motor_type, MotorControlMode mode, 
                                            float target_value)
{
    WITH_SEMAPHORE(_queue_semaphore);  // 🔒 加锁保护
    
    // Step 1: 检查队列是否已满
    if (_queue_full()) {
        _commands_dropped++;           // 增加丢弃计数
        // 记录日志并返回
        AP::logger().Write_MessageF("CAN_TX_QUEUE: Command dropped - queue full");
        return;
    }
    
    // Step 2: 在队列尾部添加新命令
    MotorCommand &cmd = _command_queue[_queue_tail];  // 获取尾部位置
    cmd.can_id = can_id;
    cmd.motor_id = motor_id;
    cmd.motor_type = motor_type;
    cmd.mode = mode;
    cmd.target_value = target_value;
    cmd.timestamp_us = AP_HAL::micros64();
    cmd.processed = false;              // 标记为未处理
    
    // Step 3: 更新队列指针和计数
    _advance_tail();                    // 尾指针向前移动
    _queue_count++;                     // 队列计数增加
}

// 尾指针移动（环形缓冲区）
void CAN_Robot_Tx_Queue::_advance_tail(void)
{
    _queue_tail = (_queue_tail + 1) % QUEUE_SIZE;  // 环形移动
}
```

### 2.3 Push过程图解
```
初始状态：
[0][1][2][3][4]...[99]
 ↑               
head=tail=0, count=0

Push第一条消息：
[A][1][2][3][4]...[99]
 ↑  ↑               
head=0, tail=1, count=1

Push第二条消息：
[A][B][2][3][4]...[99]
 ↑     ↑               
head=0, tail=2, count=2
```

## 3. 读取队列消息 (Message Reading)

### 3.1 消息读取调用链
```
AP_DroneCAN::robot_can_tx_loop() (消费者线程)
    ↓
queue->get_next_command(cmd)
    ↓
CAN_Robot_Tx_Queue::get_next_command()
```

### 3.2 具体读取过程
```cpp
bool CAN_Robot_Tx_Queue::get_next_command(MotorCommand &cmd)
{
    WITH_SEMAPHORE(_queue_semaphore);  // 🔒 加锁保护
    
    // Step 1: 检查队列是否为空
    if (_queue_empty()) {
        return false;  // 无消息可读
    }
    
    // Step 2: 从队列头部读取命令（注意：只读取，不删除）
    cmd = _command_queue[_queue_head];  // 复制命令内容
    return true;
}

// 检查队列是否为空
bool CAN_Robot_Tx_Queue::_queue_empty(void) const
{
    return _queue_count == 0;
}
```

### 3.3 ⚠️ 重要：读取 ≠ 删除
- `get_next_command()` **只读取消息，不删除消息**
- 消息仍然保留在队列中，等待处理完成后再删除
- 这样设计确保了**失败重试**的能力

## 4. 消息删除机制 (Message Deletion)

### 4.1 删除时机
**只有在CAN消息成功发送后，才会删除队列中的消息：**

```cpp
void AP_DroneCAN::robot_can_tx_loop(void)
{
    while (true) {
        CAN_Robot_Tx_Queue::MotorCommand cmd;
        
        // Step 1: 读取消息（不删除）
        if (queue->get_next_command(cmd)) {
            
            // Step 2: 创建CAN帧
            AP_HAL::CANFrame frame = CAN_Robot_Tx_Process::create_motor_frame(...);
            
            // Step 3: 发送CAN帧
            if (write_aux_frame(frame, 10 * 1000)) {
                // ✅ 发送成功，才删除消息
                queue->mark_command_processed();
            } else {
                // ❌ 发送失败，消息保留，下次重试
                debug_dronecan(AP_CANManager::LOG_ERROR, "Robot CAN TX failed");
            }
        }
    }
}
```

### 4.2 具体删除过程
```cpp
void CAN_Robot_Tx_Queue::mark_command_processed(void)
{
    WITH_SEMAPHORE(_queue_semaphore);  // 🔒 加锁保护
    
    if (!_queue_empty()) {
        // Step 1: 标记当前消息为已处理
        _command_queue[_queue_head].processed = true;
        
        // Step 2: 移动头指针（实际删除）
        _advance_head();
        
        // Step 3: 更新计数器
        _queue_count--;
        _commands_sent++;
    }
}

// 头指针移动（环形缓冲区）
void CAN_Robot_Tx_Queue::_advance_head(void)
{
    _queue_head = (_queue_head + 1) % QUEUE_SIZE;  // 环形移动
}
```

### 4.3 删除过程图解
```
处理前：
[A][B][C][3][4]...[99]
 ↑        ↑               
head=0, tail=3, count=3

处理消息A成功后：
[A][B][C][3][4]...[99]
    ↑     ↑               
head=1, tail=3, count=2  (A被"删除"，实际是head指针移动)

处理消息B成功后：
[A][B][C][3][4]...[99]
       ↑  ↑               
head=2, tail=3, count=1  (B被"删除")
```

## 5. 队列不会无限增长的原因

### 5.1 **自动平衡机制**
1. **生产速度控制**：电机控制不会无限快速地产生命令
2. **消费速度保证**：`robot_can_tx_loop`持续运行，快速处理消息
3. **容量限制**：队列最大100条消息，满了会丢弃新消息

### 5.2 **队列满时的处理**
```cpp
if (_queue_full()) {
    _commands_dropped++;  // 统计丢弃的消息
    // 记录警告日志
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAN_TX_QUEUE: Command dropped - queue full");
    return;  // 直接返回，不添加新消息
}

bool CAN_Robot_Tx_Queue::_queue_full(void) const
{
    return _queue_count >= QUEUE_SIZE;  // 达到100条就认为满了
}
```

### 5.3 **监控机制**
```cpp
void CAN_Robot_Tx_Queue::log_status(void)
{
    // 每5秒记录一次队列状态
    AP::logger().Write_MessageF("CAN_TX_QUEUE: Size:%u Sent:%u Dropped:%u", 
                               _queue_count, _commands_sent, _commands_dropped);
}
```

## 6. 顺序保证机制 (Order Guarantee)

### 6.1 **FIFO严格保证**
```
队列操作：
Push: _queue_tail 位置添加 → tail++
Pop:  _queue_head 位置读取 → head++ (删除时)

顺序：先入先出 (First In, First Out)
```

### 6.2 **多次访问的安全性**

#### A. **单消费者模式**
- 只有一个`robot_can_tx_loop`线程在消费队列
- 不存在多个消费者竞争的问题

#### B. **原子性操作**
```cpp
// 读取 → 处理 → 删除 是原子性的
if (queue->get_next_command(cmd)) {           // 1️⃣ 读取
    if (write_aux_frame(frame, timeout)) {    // 2️⃣ 发送
        queue->mark_command_processed();      // 3️⃣ 删除
    }
    // 如果发送失败，消息保留，下次重试
}
```

#### C. **信号量保护**
```cpp
// 所有队列操作都有信号量保护
void CAN_Robot_Tx_Queue::queue_motor_command(...) {
    WITH_SEMAPHORE(_queue_semaphore);  // 🔒
    // ... 操作队列
}

bool CAN_Robot_Tx_Queue::get_next_command(...) {
    WITH_SEMAPHORE(_queue_semaphore);  // 🔒
    // ... 操作队列
}

void CAN_Robot_Tx_Queue::mark_command_processed(...) {
    WITH_SEMAPHORE(_queue_semaphore);  // 🔒
    // ... 操作队列
}
```

#### D. **失败重试机制**
```cpp
// 发送失败的消息不会丢失
if (write_aux_frame(frame, 10 * 1000)) {
    queue->mark_command_processed();  // ✅ 成功才删除
} else {
    // ❌ 失败时消息保留在队列中
    // 下一次循环会重新尝试发送同一条消息
}
```

### 6.3 **顺序图示例**
```
时间线：
T1: Push消息A → Queue: [A]
T2: Push消息B → Queue: [A,B]  
T3: Push消息C → Queue: [A,B,C]
T4: 读取A，发送成功，删除A → Queue: [B,C]
T5: 读取B，发送失败，B保留 → Queue: [B,C]
T6: 读取B，发送成功，删除B → Queue: [C]
T7: 读取C，发送成功，删除C → Queue: []

结果：消息按照A→B→C的顺序处理，即使B失败重试也不影响顺序
```

## 7. 总结

### 7.1 **队列特点**
- ✅ **线程安全**：信号量保护所有操作
- ✅ **FIFO顺序**：严格的先进先出
- ✅ **失败重试**：发送失败的消息会重试
- ✅ **容量控制**：最大100条消息，防止内存溢出
- ✅ **实时监控**：统计发送成功/失败数量

### 7.2 **关键设计点**
1. **读写分离**：读取不删除，成功发送才删除
2. **原子性操作**：读取→发送→删除是原子性的
3. **环形缓冲区**：高效的内存使用
4. **单例模式**：全局唯一的队列实例

### 7.3 **回答你的问题**
- **队列如何建立**：单例模式 + 环形数组 + 指针管理
- **如何Push消息**：尾指针位置添加，移动尾指针，增加计数
- **如何读取消息**：从头指针位置读取，但不删除
- **消息会一直增加吗**：不会，有容量限制和自动消费机制
- **删除代码在哪**：`mark_command_processed()`函数
- **删除顺序对吗**：对的，严格FIFO，有完整的保护机制
- **多次访问会乱序吗**：不会，有信号量保护和原子性操作保证 