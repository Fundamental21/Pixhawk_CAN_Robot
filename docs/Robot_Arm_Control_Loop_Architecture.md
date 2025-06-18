# Robot Arm Control Loop 主控程序架构分析

## 概述

`robot_arm_control_loop()` 是机器人手臂的100Hz主控制循环，负责协调轨迹规划、电机控制、CAN通信等核心功能。本文档详细分析其程序结构、模块调用关系和执行流程。

## 系统架构

graph TD
    A["robot_arm_control_loop()<br/>100Hz主控制循环"] --> B{arm_initialized?}
    B -->|false| C["return 退出"]
    B -->|true| D["CAN Tx初始化检查<br/>tx_initialized?"]
    D -->|false| E["CAN_Robot_Tx_Queue::init()<br/>发送队列初始化"]
    D -->|true| F["process_can_rx_messages()<br/>处理CAN接收消息"]
    E --> F
    
    F --> G["轨迹生成阶段<br/>arm_current_point < 50?"]
    G -->|true| H["读取预定义关节角度<br/>predefined_joints[arm_current_point]"]
    H --> I["转换角度单位<br/>弧度转度"]
    I --> J["for loop: 6个关节电机<br/>MIT_Motor::queue_push()"]
    J --> K["arm_current_point++"]
    K --> L["电机控制循环开始"]
    G -->|false| L
    
    L --> M["for loop: 遍历6个关节电机<br/>joint_motor_list[i]"]
    M --> N["读取当前电机位置<br/>MIT_Motor::get_motor_position()"]
    N --> O{first_command &&<br/>queue.count > 0?}
    O -->|true| P["初始化梯形轨迹规划器<br/>MIT_Motor::trapezoid_init()"]
    P --> Q["设置first_command = false"]
    Q --> R["检查队列状态<br/>is_last = queue.count == 0"]
    O -->|false| R
    
    R --> S["更新梯形轨迹<br/>MIT_Motor::trapezoid_update()"]
    S --> T{轨迹完成 &&<br/>还有目标点?}
    T -->|true| U["初始化新轨迹<br/>MIT_Motor::trapezoid_init()"]
    U --> V["角度限制<br/>±180度"]
    T -->|false| V
    
    V --> W["设置调试目标值<br/>switch case 0-5"]
    W --> X["电机控制处理<br/>MIT_Motor::MotorControl_Handler()"]
    X --> Y{遍历完成?}
    Y -->|false| M
    Y -->|true| Z["函数结束"]
    
    subgraph "CAN接收处理子模块"
        F1["process_can_rx_messages()"] --> F2{rx_initialized?}
        F2 -->|false| F3["CAN_Robot_Rx_Queue::init()<br/>CAN_Robot_Rx_Process::init()"]
        F2 -->|true| F4["获取接收处理器单例<br/>CAN_Robot_Rx_Process::get_singleton()"]
        F3 --> F4
        F4 --> F5["处理所有接收消息<br/>process_all_rx_messages()"]
    end
    
    style A fill:#e1f5fe,stroke:#01579b
    style F fill:#f3e5f5,stroke:#4a148c
    style L fill:#e8f5e8,stroke:#1b5e20
    style X fill:#fff3e0,stroke:#e65100

## CAN rx, tx msg flow
graph TB
    subgraph "主控制系统"
        A["robot_arm_control_loop()<br/>100Hz主循环"]
        B["电机状态获取<br/>motor_positions[6]<br/>motor_velocities[6]<br/>motor_currents[6]"]
        C["电机控制指令<br/>target_value设置"]
    end
    
    subgraph "TX发送路径"
        D["MotorControl_Handler()"]
        E["CAN_Robot_Tx_Queue<br/>循环缓冲区"]
        F["CAN_Robot_Tx_Process<br/>create_motor_frame()"]
    end
    
    subgraph "RX接收路径"
        G["CAN_Robot_Rx_Queue<br/>循环缓冲区"]
        H["CAN_Robot_Rx_Process<br/>decode_mit_feedback()"]
        I["MIT_Motor_Feedback<br/>数据存储"]
    end
    
    subgraph "硬件接口"
        J["AP_Canard_iface<br/>DroneCAN接口"]
        K["CAN1/CAN2硬件"]
    end
    
    subgraph "物理设备"
        L["MIT电机1-6<br/>CAN ID: 0x01-0x06"]
    end
    
    A --> C
    A --> B
    C --> D
    D --> E
    E --> F
    F --> J
    J --> K
    K --> L
    
    L --> K
    K --> J
    J --> G
    G --> H
    H --> I
    I --> B
    
    style A fill:#E6F3FF
    style E fill:#FFF2E6
    style G fill:#FFF2E6
    style I fill:#F0F8E6
    style L fill:#FFE6E6

### 主控制循环概览

```cpp
void Rover::robot_arm_control_loop()  // 100Hz调度执行
```

**调度信息**：
- **执行频率**: 100Hz (每10ms执行一次)
- **调度优先级**: 1 (最高优先级)
- **预期执行时间**: 200微秒

## 程序结构分析

### 1. 初始化检查阶段

```cpp
if (!arm_initialized) {
    return;  // 手臂未初始化则退出
}
```

**功能**: 确保机器人手臂已完成基本初始化后才开始控制循环。

### 2. CAN发送队列初始化

```cpp
static bool tx_initialized = false;
if (!tx_initialized) {
    CAN_Robot_Tx_Queue::init();  // 单例模式初始化
    tx_initialized = true;
}
```

**功能**: 
- 单次初始化CAN发送队列
- 采用单例模式，全局唯一实例
- 负责所有电机控制指令的排队发送

### 3. CAN接收消息处理

```cpp
process_can_rx_messages();
```

**功能**: 处理来自CAN总线的反馈消息，包括电机状态、位置、速度等信息。

### 4. 轨迹生成阶段

```cpp
if (arm_current_point < 50) {
    // 从预定义轨迹中读取目标角度
    memcpy(input_angles.angles, predefined_joints[arm_current_point], ...);
    
    // 为每个关节电机添加目标位置到队列
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        float motor_pos = input_angles.angles[i] * (180.0f / M_PI);
        MIT_Motor::queue_push(&m->queue, motor_pos);
    }
    arm_current_point++;
}
```

**特点**:
- **轨迹源**: 预定义的关节角度数组 `predefined_joints`
- **角度转换**: 弧度转换为度 (×180/π)
- **队列管理**: 每个电机维护独立的位置队列
- **执行次数**: 50个预定义点，执行完成后停止

### 5. 电机控制循环

这是主控制循环的核心部分，对6个关节电机逐一进行控制：

```cpp
for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
    MotorInstance* m = joint_motor_list[i];
    // 电机控制逻辑...
}
```

#### 5.1 位置反馈读取

```cpp
float current_pos = MIT_Motor::get_motor_position(m->can_id, m->motor_id);
```

#### 5.2 梯形轨迹规划器

```cpp
// 首次命令初始化
if (m->first_command && m->queue.count > 0) {
    MIT_Motor::trapezoid_init(&m->planner, current_pos, MIT_Motor::queue_pop(&m->queue));
    m->first_command = false;
}

// 轨迹更新
MIT_Motor::trapezoid_update(&m->planner, is_last);

// 新轨迹段处理
if (m->planner.is_terminated && m->queue.count > 0) {
    MIT_Motor::trapezoid_init(&m->planner, m->planner.current_ref, MIT_Motor::queue_pop(&m->queue));
}
```

#### 5.3 调试目标值设置

```cpp
switch (i) {
    case 0: m->target_value = -3.2f + debug_current1; break;
    case 1: m->target_value = 6.2923f + debug_current2; break;
    // ... 其他关节
}
```

#### 5.4 电机控制指令发送

```cpp
MIT_Motor::MotorControl_Handler(m);
```

## 模块调用关系

### 核心模块依赖

```
robot_arm_control_loop()
├── CAN_Robot_Tx_Queue (发送队列)
│   ├── init() - 单例初始化
│   └── queue_motor_command() - 指令排队
├── process_can_rx_messages() (接收处理)
│   ├── CAN_Robot_Rx_Queue - 接收队列
│   └── CAN_Robot_Rx_Process - 消息解析
└── MIT_Motor (电机控制命名空间)
    ├── get_motor_position() - 位置读取
    ├── queue_push() - 队列操作
    ├── queue_pop() - 队列操作
    ├── trapezoid_init() - 轨迹初始化
    ├── trapezoid_update() - 轨迹更新
    └── MotorControl_Handler() - 控制处理
```

### 数据结构关系

```
MotorInstance (电机实例)
├── can_id, motor_id (CAN标识)
├── mode, target_value (控制参数)
├── PositionQueue queue (位置队列)
└── TrapezoidPlanner planner (轨迹规划器)

TrapezoidPlanner (梯形轨迹规划器)
├── current_ref (当前参考位置)
├── current_vel (当前速度)
├── target_pos (目标位置)
├── max_velocity, max_accel (限制参数)
└── is_terminated (完成标志)
```

## 执行时序分析

### 时序特性

| 阶段 | 执行频率 | 说明 |
|------|----------|------|
| **初始化检查** | 每次调用 | 轻量级检查 |
| **CAN Tx初始化** | 单次 | 程序启动时执行一次 |
| **CAN消息处理** | 100Hz | 处理接收到的反馈 |
| **轨迹生成** | 100Hz | 直到50个点用完 |
| **电机控制** | 100Hz × 6电机 | 每个电机独立控制 |

### 处理优先级

1. **CAN消息处理** - 优先处理反馈信息
2. **轨迹生成** - 添加新的目标点
3. **轨迹规划** - 计算平滑轨迹
4. **电机控制** - 发送控制指令

## 关键设计特点

### 1. 分层架构设计

- **应用层**: robot_arm_control_loop() 主控制逻辑
- **中间层**: MIT_Motor命名空间提供电机抽象
- **通信层**: CAN队列系统处理底层通信

### 2. 队列缓冲机制

- **位置队列**: 每个电机维护50个位置的FIFO队列
- **CAN发送队列**: 全局100条指令的发送缓冲
- **CAN接收队列**: 全局100条消息的接收缓冲

### 3. 梯形轨迹规划

```cpp
// 轨迹参数
#define MAX_VELOCITY     10.0f    // deg/s
#define MAX_ACCEL        20.0f    // deg/s^2
#define CONTROL_PERIOD   0.01f    // 10ms
```

**特点**:
- 平滑的速度曲线
- 加速度限制保护机械结构
- 连续轨迹段无缝连接

### 4. 实时性保证

- **固定周期**: 100Hz严格调度
- **优先级**: 最高调度优先级 (priority=1)
- **时间预算**: 200微秒执行时间限制

## 扩展接口

### 夹爪控制接口（已预留）

```cpp
// 夹爪电机控制 (当前注释掉)
// MotorInstance* gm = grasper_motor_list[0];
// gm->target_value = debug_current7;
// handle_kegu_motor(gm);  // KEGU电机控制
```

### 调试接口

```cpp
// 20Hz调试日志记录 (当前注释掉)
// debug_buffer[debug_counter] = joint_motor_list[4]->planner.current_ref;
// real_buffer[debug_counter] = debug_position;
```

## 性能考虑

### 计算复杂度

- **轨迹生成**: O(n) n=6个电机
- **轨迹规划**: O(1) 每个电机
- **CAN处理**: O(m) m=队列中消息数

### 内存使用

- **位置队列**: 6电机 × 50位置 × 4字节 = 1.2KB
- **电机实例**: 6电机 × sizeof(MotorInstance)
- **CAN缓冲**: 发送+接收队列约8KB

## 故障处理

### 安全机制

1. **角度限制**: ±180度范围限制
2. **初始化检查**: arm_initialized标志
3. **队列保护**: 防止队列溢出
4. **CAN超时**: 通信超时处理

### 调试支持

- 每个关节独立的调试偏移量 (debug_current1-6)
- 实时位置记录缓冲区
- 轨迹规划状态监控

## 总结

`robot_arm_control_loop()` 采用了分层、模块化的设计，实现了：

- **高实时性**: 100Hz精确控制
- **平滑运动**: 梯形轨迹规划
- **可靠通信**: CAN队列缓冲机制
- **易于扩展**: 模块化接口设计

这种架构为机器人手臂提供了稳定、精确、可扩展的控制基础。 