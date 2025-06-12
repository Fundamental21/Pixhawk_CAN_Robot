# CAN Robot TX 消息流程文档

## 1. 消息发送流程概述

整个CAN消息的发送流程分为四个主要阶段：
1. 机械臂控制循环（主程序）
2. 电机控制命令生成
3. 命令队列管理
4. CAN消息发送

## 2. 详细流程说明

### 2.1 机械臂控制循环

**文件位置**: `Rover/Rover.cpp`
```cpp
void Rover::robot_arm_control_loop(void)
{
    // 处理机械臂电机
    for (int i = 0; i < JOINT_MOTOR_COUNT; i++) {
        MotorInstance* m = &motor_instances[0][i];  // 使用CAN1的电机
        if (m->enabled) {
            // 根据控制模式更新目标值
            update_motor_target(m);
            // 处理电机控制
            MotorControl_Handler(m);
        }
    }
}
```

需要修改的部分：
1. 确保 `motor_instances` 数组正确初始化
2. 添加 `update_motor_target` 函数实现
3. 在 `Rover.h` 中添加相关变量和函数声明

### 2.2 电机控制命令生成

**文件位置**: `Rover/MIT_Motor.cpp`
```cpp
void MotorControl_Handler(MotorInstance* motor)
{
    if(!motor || !motor->enabled) return;

    // 获取队列单例
    CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
    if (!queue) return;

    // 转换控制模式
    MotorControlMode queue_mode;
    switch(motor->mode) {
        case CTRL_MODE_POSITION:
            queue_mode = CTRL_MODE_POSITION;
            break;
        case CTRL_MODE_VELOCITY:
            queue_mode = CTRL_MODE_VELOCITY;
            break;
        case CTRL_MODE_CURRENT:
            queue_mode = CTRL_MODE_CURRENT;
            break;
        default:
            return;
    }

    // 将命令加入队列
    queue->queue_motor_command(motor->can_id, motor->motor_id, 
                             motor->type == MOTOR_TYPE_MIT ? MotorType::MIT : MotorType::KEGU,
                             queue_mode, motor->target_value);
}
```

### 2.3 命令队列管理

**文件位置**: `libraries/CAN_Robot_Tx/CAN_Robot_Tx_Queue.h`
```cpp
class CAN_Robot_Tx_Queue {
public:
    // 电机命令结构体
    struct MotorCommand {
        uint8_t can_id;           // CAN总线ID
        uint8_t motor_id;         // 电机ID
        MotorType motor_type;     // 电机类型（MIT/KEGU）
        MotorControlMode mode;    // 控制模式
        float target_value;       // 目标值
        uint64_t timestamp_us;    // 时间戳
        bool processed;           // 处理状态
    };

    static CAN_Robot_Tx_Queue* get_singleton() {
        return _singleton;
    }

    void queue_motor_command(uint8_t can_id, uint8_t motor_id, 
                           MotorType type, MotorControlMode mode, 
                           float value);
    bool get_next_command(MotorCommand &cmd);

private:
    static CAN_Robot_Tx_Queue* _singleton;
    MotorCommand _queue[QUEUE_SIZE];
    uint16_t _queue_head;
    uint16_t _queue_tail;
    uint16_t _queue_count;
};
```

### 2.4 CAN消息发送

**文件位置**: `libraries/AP_DroneCAN/AP_DroneCAN.cpp`
```cpp
void AP_DroneCAN::robot_can_tx_loop(void)
{
    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        // 获取队列实例
        CAN_Robot_Tx_Queue* queue = CAN_Robot_Tx_Queue::get_singleton();
        if (!queue) {
            hal.scheduler->delay(10);
            continue;
        }

        // 处理队列中的命令
        CAN_Robot_Tx_Queue::MotorCommand cmd;
        if (queue->get_next_command(cmd)) {
            // 获取CAN帧处理器
            CAN_Robot_Tx_Process* processor = CAN_Robot_Tx_Process::get_singleton();
            if (processor) {
                // 创建CAN帧
                AP_HAL::CANFrame frame;
                if (cmd.motor_type == MotorType::MIT) {
                    frame = create_mit_motor_frame(cmd.can_id, cmd.motor_id, 
                                                 static_cast<uint8_t>(cmd.mode), 
                                                 cmd.target_value);
                } else {
                    frame = create_kegu_motor_frame(cmd.can_id, cmd.motor_id, 
                                                  static_cast<uint8_t>(cmd.mode), 
                                                  cmd.target_value);
                }
                
                // 发送CAN帧（10ms超时）
                if (!write_aux_frame(frame, 10 * 1000)) {
                    AP::logger().Write_Error(LogErrorSubsystem::CAN, 
                                           LogErrorCode::SEND_ERROR);
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ROBOT_TX: Failed to send CAN frame ID=0x%X", 
                                (unsigned)frame.id);
                }
            }
        } else {
            hal.scheduler->delay_microseconds(100);
        }
    }
}
```

## 3. 数据流程图

```mermaid
graph TD
    A[Rover::robot_arm_control_loop]
    B[MotorInstance 结构体<br>can_id<br>motor_id<br>type<br>mode<br>target_value<br>enabled]
    C[MIT_Motor::MotorControl_Handler<br>参数: MotorInstance* motor]
    D[CAN_Robot_Tx_Queue::queue_motor_command<br>参数:<br>can_id<br>motor_id<br>motor_type<br>mode<br>target_value]
    E[CAN_Robot_Tx_Queue 内部数组<br>_queue[QUEUE_SIZE]<br>类型: MotorCommand结构体]
    F[AP_DroneCAN::robot_can_tx_loop]
    G[CAN_Robot_Tx_Queue::get_next_command<br>返回: MotorCommand结构体]
    H[CAN_Robot_Tx_Process::create_mit_motor_frame<br>参数:<br>can_id<br>motor_id<br>cmd<br>value]
    I[AP_HAL::CANFrame 结构体<br>id<br>dlc=8<br>data[8]]
    J[AP_DroneCAN::write_aux_frame<br>参数:<br>frame<br>timeout_us=10000]
    K[CAN总线硬件]

    A -->|传递电机指针 m| C
    B -->|包含电机参数| C
    C -->|调用| D
    D -->|存储命令| E
    F -->|循环读取| G
    G -->|返回| E
    F -->|获取命令| H
    H -->|创建| I
    F -->|发送帧| J
    J -->|硬件发送| K

    subgraph 主循环线程
        A
        B
        C
        D
    end

    subgraph 命令队列
        E
    end

    subgraph CAN发送线程
        F
        G
        H
        I
        J
    end

    subgraph 硬件层
        K
    end

    style A fill:#f9f,stroke:#333,stroke-width:2px
    style E fill:#bbf,stroke:#333,stroke-width:2px
    style F fill:#f96,stroke:#333,stroke-width:2px
    style K fill:#bfb,stroke:#333,stroke-width:2px
```

## 数据传递过程详解：

1. **主循环到队列**:
   ```cpp
   // Rover::robot_arm_control_loop
   MotorInstance* m = &motor_instances[0][i];
   MotorControl_Handler(m);
   
   // MIT_Motor::MotorControl_Handler
   queue->queue_motor_command(
       motor->can_id,      // uint8_t
       motor->motor_id,    // uint8_t
       motor->type,        // MotorType
       queue_mode,         // MotorControlMode
       motor->target_value // float
   );
   ```

2. **队列存储**:
   ```cpp
   // CAN_Robot_Tx_Queue::queue_motor_command
   MotorCommand &cmd = _queue[_queue_tail];
   cmd.can_id = can_id;
   cmd.motor_id = motor_id;
   cmd.motor_type = type;
   cmd.mode = mode;
   cmd.target_value = value;
   cmd.timestamp_us = AP_HAL::micros64();
   cmd.processed = false;
   ```

3. **队列到CAN发送**:
   ```cpp
   // AP_DroneCAN::robot_can_tx_loop
   CAN_Robot_Tx_Queue::MotorCommand cmd;
   if (queue->get_next_command(cmd)) {
       AP_HAL::CANFrame frame = create_mit_motor_frame(
           cmd.can_id,
           cmd.motor_id,
           static_cast<uint8_t>(cmd.mode),
           cmd.target_value
       );
       write_aux_frame(frame, 10 * 1000);
   }
   ```

4. **CAN帧创建**:
   ```cpp
   // CAN_Robot_Tx_Process::create_mit_motor_frame
   AP_HAL::CANFrame frame;
   frame.id = motor_id;
   frame.dlc = 8;
   frame.data[0] = cmd;
   frame.data[1] = 0;
   frame.data[2] = (int_value >> 24) & 0xFF;
   frame.data[3] = (int_value >> 16) & 0xFF;
   frame.data[4] = (int_value >> 8) & 0xFF;
   frame.data[5] = int_value & 0xFF;
   frame.data[6] = 0;
   frame.data[7] = 0;
   ```

## 4. 需要修改的部分

### 4.1 Rover.h 修改
```cpp
class Rover : public AP_Vehicle {
public:
    // ... 其他代码 ...
    
    // 机械臂控制相关
    void robot_arm_control_loop(void);
    void update_motor_target(MotorInstance* motor);
    
private:
    // ... 其他代码 ...
    
    // 电机实例数组
    MotorInstance motor_instances[MAX_CAN_NUM][MOTORS_PER_CAN];
};
```

### 4.2 AP_DroneCAN.h 修改
```cpp
class AP_DroneCAN : public AP_CANDriver {
public:
    // ... 其他代码 ...
    
    // 机器人CAN发送线程
    void robot_can_tx_loop(void);
    
private:
    // ... 其他代码 ...
};
```

### 4.3 AP_DroneCAN.cpp 修改
在 `init` 函数中添加线程创建：
```cpp
void AP_DroneCAN::init(uint8_t driver_index, bool enable_filters)
{
    // ... 其他代码 ...

    // 创建机器人CAN发送线程
    hal.util->snprintf(_thread_name, sizeof(_thread_name), "robot_tx_%u", driver_index);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DroneCAN::robot_can_tx_loop, void),
                                    _thread_name, DRONECAN_STACK_SIZE,
                                    AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_dronecan(AP_CANManager::LOG_ERROR, "Can't create robot TX thread\n\r");
        return;
    }
}
```

## 5. 线程安全性

- 使用单例模式确保全局唯一实例
- 队列操作使用互斥锁保护
- CAN发送使用超时机制（10ms）

## 6. 错误处理

1. 队列满时：
   - 增加丢弃计数
   - 记录警告日志

2. CAN发送失败时：
   - 记录错误日志
   - 发送错误消息到地面站

3. 系统未初始化时：
   - 等待并重试
   - 避免无效操作 