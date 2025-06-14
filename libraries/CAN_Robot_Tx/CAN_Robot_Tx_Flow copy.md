# CAN Robot TX 消息流程文档

## 1. 整体架构概述

CAN机器人发送系统采用**生产者-消费者模式**，通过队列解耦控制逻辑和CAN发送，确保实时性和线程安全。

### 1.1 核心组件
- **Rover::robot_arm_control_loop**: 主控制循环（100Hz）
- **MIT_Motor::MotorControl_Handler**: 电机控制处理器
- **CAN_Robot_Tx_Queue**: 线程安全的命令队列（单例模式）
- **AP_DroneCAN::robot_can_tx_loop**: CAN发送线程
- **CAN_Robot_Tx_Process**: 统一的CAN帧创建器

### 1.2 数据流架构

```
Rover::robot_arm_control_loop (100Hz)
    ↓ [MotorInstance*]
MIT_Motor::MotorControl_Handler
    ↓ [MotorCommand]
CAN_Robot_Tx_Queue::queue_motor_command
    ↓ [队列存储]
AP_DroneCAN::robot_can_tx_loop (专用线程)
    ↓ [MotorCommand]
CAN_Robot_Tx_Process::create_motor_frame  ← 统一入口
    ↓ [AP_HAL::CANFrame]
create_mit_motor_frame / create_kegu_motor_frame
    ↓ [CAN总线]
```

## 2. 详细实现流程

### 2.1 阶段1: 主控制循环

**文件**: `Rover/Rover.cpp:280`
**频率**: 100Hz
**线程**: 主调度线程

```cpp
void Rover::robot_arm_control_loop()
{
    if (!arm_initialized) return;

    // 500Hz trajectory generation
    if (arm_current_point < 50) {
        memcpy(input_angles.angles, predefined_joints[arm_current_point], 
               sizeof(predefined_joints[arm_current_point]));
        
        for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
            MotorInstance* m = joint_motor_list[i];
            float motor_pos = input_angles.angles[i] * (180.0f / M_PI);
            MIT_Motor::queue_push(&m->queue, motor_pos);
        }
        arm_current_point++; 
    }  

    // 100Hz motor control
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        MotorInstance* m = joint_motor_list[i];
        
        // 轨迹规划和控制逻辑
        float current_pos = MIT_Motor::get_motor_position(m->can_id, m->motor_id);
        
        if (m->first_command && m->queue.count > 0) {
            MIT_Motor::trapezoid_init(&m->planner, current_pos, 
                                    MIT_Motor::queue_pop(&m->queue));
            m->first_command = false;
        }

        bool is_last = (m->queue.count == 0);
        MIT_Motor::trapezoid_update(&m->planner, is_last);
        
        if (m->planner.is_terminated && m->queue.count > 0) {
            MIT_Motor::trapezoid_init(&m->planner, m->planner.current_ref,
                        MIT_Motor::queue_pop(&m->queue));
        }
        
        // 设置目标值并发送命令
        m->target_value = m->planner.current_ref; // 或其他控制逻辑
        MIT_Motor::MotorControl_Handler(m);  // 关键调用点
    }
}
```

**关键数据结构**:
```cpp
struct MotorInstance {
    uint8_t can_id;              // CAN总线ID
    uint8_t motor_id;            // 电机ID  
    MotorControlMode mode;       // 控制模式
    float target_value;          // 目标值
    bool enabled;                // 使能状态
    MotorType type;              // 电机类型
    bool first_command;          // 首次命令标志
    PositionQueue queue;         // 位置队列
    TrapezoidPlanner planner;    // 轨迹规划器
};
```

### 2.2 阶段2: 电机控制处理

**文件**: `Rover/MIT_Motor.cpp:130`
**调用**: 来自 robot_arm_control_loop

```cpp
void MotorControl_Handler(MotorInstance* motor)
{
    if(!motor || !motor->enabled) return;

    // 获取全局队列单例
    CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
    if (!queue) return;

    // 类型转换: 本地enum -> 队列enum
    MotorControlMode queue_mode = motor->mode;

    // 将命令推入线程安全队列
    queue->queue_motor_command(
        motor->can_id,      // CAN总线ID
        motor->motor_id,    // 电机ID
        motor->type == MOTOR_TYPE_MIT ? MotorType::MIT : MotorType::KEGU,
        queue_mode,         // 控制模式
        motor->target_value // 目标值
    );
    
    // 可选: 请求状态反馈
    if (motor->type == MOTOR_TYPE_MIT) {
        queue->queue_motor_command(motor->can_id, motor->motor_id, 
                                 MotorType::MIT, MotorControlMode::CTRL_MODE_CURRENT, 0);
    }
}
```

### 2.3 阶段3: 命令队列管理

**文件**: `libraries/CAN_Robot_Tx/CAN_Robot_Tx_Queue.cpp`
**模式**: 单例模式 + 循环缓冲区

```cpp
void CAN_Robot_Tx_Queue::queue_motor_command(uint8_t can_id, uint8_t motor_id, 
                                           MotorType motor_type, MotorControlMode mode, 
                                           float target_value)
{
    // 检查队列是否已满
    if (_queue_count >= QUEUE_SIZE) {
        _dropped_commands++;
        AP::logger().Write_MessageF("CAN_TX_QUEUE: Queue full, dropped command");
        return;
    }

    // 创建命令结构体
    MotorCommand &cmd = _queue[_queue_tail];
    cmd.can_id = can_id;
    cmd.motor_id = motor_id;
    cmd.motor_type = motor_type;
    cmd.mode = mode;
    cmd.target_value = target_value;
    cmd.timestamp_us = AP_HAL::micros64();
    cmd.processed = false;

    // 更新队列指针（线程安全）
    _queue_tail = (_queue_tail + 1) % QUEUE_SIZE;
    _queue_count++;
    
    _total_commands++;
}

bool CAN_Robot_Tx_Queue::get_next_command(MotorCommand &cmd)
{
    if (_queue_count == 0) return false;

    // 找到下一个未处理的命令
    for (uint16_t i = 0; i < QUEUE_SIZE; i++) {
        uint16_t index = (_queue_head + i) % QUEUE_SIZE;
        if (!_queue[index].processed) {
            cmd = _queue[index];
            return true;
        }
    }
    return false;
}

void CAN_Robot_Tx_Queue::mark_command_processed()
{
    if (_queue_count == 0) return;

    // 标记队列头部命令为已处理
    _queue[_queue_head].processed = true;
    _queue_head = (_queue_head + 1) % QUEUE_SIZE;
    _queue_count--;
}
```

**队列数据结构**:
```cpp
struct MotorCommand {
    uint8_t can_id;               // CAN总线ID
    uint8_t motor_id;             // 电机ID
    MotorType motor_type;         // 电机类型 (MIT/KEGU)
    MotorControlMode mode;        // 控制模式
    float target_value;           // 目标值
    uint64_t timestamp_us;        // 时间戳（微秒）
    bool processed;               // 处理状态标志
};
```

### 2.4 阶段4: CAN发送线程

**文件**: `libraries/AP_DroneCAN/AP_DroneCAN.cpp:2054`
**线程**: 专用线程 "robot_tx_N"
**优先级**: AP_HAL::Scheduler::PRIORITY_CAN

```cpp
void AP_DroneCAN::robot_can_tx_loop(void)
{
    while (true) {
        // 等待系统初始化
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        // 获取队列单例
        CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
        if (!queue) {
            hal.scheduler->delay(10);
            continue;
        }

        // 处理队列中的命令
        CAN_Robot_Tx_Queue::MotorCommand cmd;
        if (queue->get_next_command(cmd)) {
            // 使用统一方法创建CAN帧
            AP_HAL::CANFrame frame = CAN_Robot_Tx_Process::create_motor_frame(
                cmd.can_id, 
                cmd.motor_id, 
                cmd.motor_type, 
                cmd.mode, 
                cmd.target_value
            );
            
            // 发送CAN帧（10ms超时）
            if (frame.dlc > 0) {
                if (write_aux_frame(frame, 10 * 1000)) {
                    // 发送成功，标记命令为已处理
                    queue->mark_command_processed();
                    
                    debug_dronecan(AP_CANManager::LOG_DEBUG, 
                                 "Robot CAN TX: ID=0x%X Mode=%u Value=%.2f\\n", 
                                 (unsigned)cmd.can_id, (unsigned)cmd.mode, 
                                 (double)cmd.target_value);
                } else {
                    // 发送失败
                    debug_dronecan(AP_CANManager::LOG_ERROR, 
                                 "Robot CAN TX failed: ID=0x%X\\n", 
                                 (unsigned)cmd.can_id);
                }
            } else {
                // 无效帧，标记为已处理避免死循环
                queue->mark_command_processed();
                debug_dronecan(AP_CANManager::LOG_ERROR, 
                             "Invalid motor frame: Type=%u Mode=%u\\n", 
                             (unsigned)cmd.motor_type, (unsigned)cmd.mode);
            }
        } else {
            // 队列为空，短暂休眠
            hal.scheduler->delay_microseconds(100);
        }
    }
}
```

### 2.5 阶段5: 统一CAN帧创建

**文件**: `libraries/CAN_Robot_Tx/CAN_Robot_Tx_Process.cpp:384`
**作用**: 统一入口，消除代码重复

```cpp
AP_HAL::CANFrame CAN_Robot_Tx_Process::create_motor_frame(uint8_t can_id, uint8_t motor_id, 
                                                         MotorType motor_type, MotorControlMode mode, 
                                                         float target_value)
{
    AP_HAL::CANFrame frame;
    
    if (motor_type == MotorType::MIT) {
        switch (mode) {
            case MotorControlMode::CTRL_MODE_POSITION:
                frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_POSITION, target_value);
                break;
            case MotorControlMode::CTRL_MODE_VELOCITY:
                frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_SPEED, target_value);
                break;
            case MotorControlMode::CTRL_MODE_CURRENT:
                frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_CURRENT, target_value);
                break;
            default:
                memset(&frame, 0, sizeof(frame));  // 返回空帧
                break;
        }
    } else if (motor_type == MotorType::KEGU) {
        switch (mode) {
            case MotorControlMode::CTRL_MODE_INIT:
                frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_INIT, target_value);
                break;
            case MotorControlMode::CTRL_MODE_ENABLE_CUR:
                frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_ENABLE_CUR, target_value);
                break;
            case MotorControlMode::CTRL_MODE_CURRENT:
                frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_SET_CUR, target_value);
                break;
            default:
                memset(&frame, 0, sizeof(frame));  // 返回空帧
                break;
        }
    } else {
        memset(&frame, 0, sizeof(frame));  // 不支持的电机类型
    }
    
    return frame;
}
```

### 2.6 阶段6: 具体CAN帧生成

**MIT电机帧格式**:
```cpp
AP_HAL::CANFrame create_mit_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame;
    frame.id = motor_id;        // CAN ID = 电机ID
    frame.dlc = 8;              // 数据长度8字节
    
    // 根据命令类型转换浮点值为整数
    int32_t int_value;
    switch (cmd) {
        case MIT_CMD_POSITION:  // 0x01
            int_value = static_cast<int32_t>(value * 100.0f);  // 0.01度分辨率
            break;
        case MIT_CMD_SPEED:     // 0x02
            int_value = static_cast<int32_t>(value * 100.0f);  // 0.01RPM分辨率
            break;
        case MIT_CMD_CURRENT:   // 0x03
            int_value = static_cast<int32_t>(value * 1000.0f); // mA分辨率
            break;
        default:
            int_value = static_cast<int32_t>(value);
    }
    
    // 填充CAN帧数据
    frame.data[0] = cmd;                        // 命令字节
    frame.data[1] = 0;                          // 保留
    frame.data[2] = (int_value >> 24) & 0xFF;   // 数据高字节
    frame.data[3] = (int_value >> 16) & 0xFF;
    frame.data[4] = (int_value >> 8) & 0xFF;
    frame.data[5] = int_value & 0xFF;           // 数据低字节
    frame.data[6] = 0;                          // 保留
    frame.data[7] = 0;                          // 保留
    
    return frame;
}
```

**KEGU电机帧格式**:
```cpp
AP_HAL::CANFrame create_kegu_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame;
    frame.id = motor_id;
    frame.dlc = 8;
    
    int32_t int_value = static_cast<int32_t>(value);
    
    frame.data[0] = cmd;                        // 命令字节
    frame.data[1] = (int_value >> 24) & 0xFF;   // 数据高字节
    frame.data[2] = (int_value >> 16) & 0xFF;
    frame.data[3] = (int_value >> 8) & 0xFF;
    frame.data[4] = int_value & 0xFF;           // 数据低字节
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    return frame;
}
```

## 3. 数据流程图

```mermaid
graph TD
    A[Rover::robot_arm_control_loop<br/>100Hz主循环]
    B[MotorInstance结构体<br/>包含: can_id, motor_id, type, mode, target_value]
    C[MIT_Motor::MotorControl_Handler<br/>参数: MotorInstance* motor]
    D[CAN_Robot_Tx_Queue::queue_motor_command<br/>线程安全队列操作]
    E[MotorCommand队列<br/>循环缓冲区_queue[QUEUE_SIZE]]
    F[AP_DroneCAN::robot_can_tx_loop<br/>专用CAN发送线程]
    G[CAN_Robot_Tx_Queue::get_next_command<br/>获取未处理命令]
    H[CAN_Robot_Tx_Process::create_motor_frame<br/>统一CAN帧创建入口]
    I[create_mit_motor_frame<br/>MIT电机专用帧生成]
    J[create_kegu_motor_frame<br/>KEGU电机专用帧生成]
    K[AP_HAL::CANFrame<br/>标准CAN帧结构]
    L[AP_DroneCAN::write_aux_frame<br/>10ms超时发送]
    M[CAN总线硬件<br/>物理层传输]
    N[CAN_Robot_Tx_Queue::mark_command_processed<br/>标记命令已处理]

    A -->|传递电机实例| C
    B -->|包含所有电机参数| C
    C -->|调用队列命令| D
    D -->|存储到循环缓冲区| E
    F -->|循环检查队列| G
    G -->|返回待处理命令| F
    F -->|调用统一创建方法| H
    H -->|MIT电机类型| I
    H -->|KEGU电机类型| J
    I -->|生成MIT帧| K
    J -->|生成KEGU帧| K
    F -->|发送CAN帧| L
    L -->|成功发送| N
    L -->|硬件传输| M
    N -->|更新队列状态| E

    subgraph 主线程
        A
        B
        C
        D
    end

    subgraph 线程安全队列
        E
    end

    subgraph CAN发送线程
        F
        G
        L
        N
    end

    subgraph CAN帧创建模块
        H
        I
        J
        K
    end

    subgraph 硬件层
        M
    end

    style A fill:#e1f5fe
    style F fill:#f3e5f5
    style H fill:#e8f5e8
    style M fill:#fff3e0
```

## 4. 电机类型与命令对照表

### 4.1 MIT电机命令
| 命令类型 | 命令码 | 控制模式 | 数据格式 | 说明 |
|---------|-------|---------|---------|------|
| 位置控制 | 0x01 | CTRL_MODE_POSITION | value * 100 | 0.01度分辨率 |
| 速度控制 | 0x02 | CTRL_MODE_VELOCITY | value * 100 | 0.01RPM分辨率 |
| 电流控制 | 0x03 | CTRL_MODE_CURRENT | value * 1000 | mA分辨率 |
| 设置ID | 0x05 | - | motor_id | 设置电机ID |

### 4.2 KEGU电机命令
| 命令类型 | 命令码 | 控制模式 | 数据格式 | 说明 |
|---------|-------|---------|---------|------|
| 初始化 | 0x01 | CTRL_MODE_INIT | int32_t | 电机初始化 |
| 使能电流 | 0x02 | CTRL_MODE_ENABLE_CUR | int32_t | 使能电流控制 |
| 设置电流 | 0x03 | CTRL_MODE_CURRENT | int32_t | 设置目标电流 |

## 5. 线程安全与性能

### 5.1 线程安全机制
- **单例模式**: 确保全局唯一的队列实例
- **原子操作**: 队列指针更新使用原子操作
- **循环缓冲区**: 避免动态内存分配
- **状态标志**: 使用 `processed` 标志避免重复处理

### 5.2 性能优化
- **统一入口**: `create_motor_frame` 消除代码重复
- **预分配内存**: 队列使用固定大小数组
- **快速路径**: 空队列时快速退出
- **超时机制**: CAN发送使用10ms超时避免阻塞

### 5.3 错误处理
- **队列满处理**: 丢弃新命令并记录统计
- **发送失败**: 记录错误日志和GCS消息
- **无效帧**: 标记为已处理避免死循环
- **系统未初始化**: 等待重试机制

## 6. 配置与调试

### 6.1 关键参数
```cpp
#define QUEUE_SIZE 100              // 队列大小
#define JOINT_MOTOR_COUNT 6         // 关节电机数量
#define DRONECAN_STACK_SIZE 8192    // 线程栈大小
#define CAN_TX_TIMEOUT_MS 10        // CAN发送超时
```

### 6.2 日志记录
- 队列状态：总命令数、丢弃数、当前长度
- CAN发送：成功/失败次数、超时次数  
- 电机状态：位置、速度、电流反馈
- 错误信息：系统级错误和警告

### 6.3 监控点
- 队列利用率：`_queue_count / QUEUE_SIZE`
- 命令处理延迟：`当前时间 - timestamp_us`
- CAN总线负载：发送频率统计
- 线程CPU占用：通过系统监控

## 7. 扩展与维护

### 7.1 添加新电机类型
1. 在 `MotorType` 枚举中添加新类型
2. 在 `create_motor_frame` 中添加新的分支
3. 实现对应的 `create_xxx_motor_frame` 函数
4. 更新命令码定义

### 7.2 性能调优
- 根据实际负载调整 `QUEUE_SIZE`
- 根据网络延迟调整 `CAN_TX_TIMEOUT_MS`
- 监控队列深度决定是否需要优先级队列
- 考虑批量发送减少系统调用开销

### 7.3 故障排查
1. **队列满**: 检查生产速度是否超过消费速度
2. **CAN发送失败**: 检查硬件连接和总线负载
3. **命令丢失**: 检查队列大小和处理逻辑
4. **延迟过高**: 检查线程优先级和系统负载 