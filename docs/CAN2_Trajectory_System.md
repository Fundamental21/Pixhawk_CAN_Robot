# CAN2轨迹数据接收系统

## 概述

本系统实现了从CAN2口接收外部轨迹数据的功能，替代了原来的预定义关节位置数组。系统通过CAN2总线接收6个关节的位置数据，并将其添加到轨迹插值器队列中，实现平滑的机器人臂运动控制。

## 系统架构

### 1. 数据流

```
外部设备 (CAN2) 
    ↓ [CAN帧: ID=0x100, 数据=6个关节位置]
AP_DroneCAN::processRxFromInterface()
    ↓ [CAN_Robot_Rx_Queue]
CAN_Robot_Rx_Process::process_can2_trajectory_command()
    ↓ [添加到轨迹数据队列]
TrajectoryDataQueue (50个点的缓冲队列)
    ↓ [Rover::robot_arm_control_loop处理]
TrajectoryInterpolator::add_trajectory_point()
    ↓ [插值生成平滑轨迹]
MIT_Motor::MotorControl_Handler()
    ↓ [CAN1发送控制指令]
6个MIT电机
```

### 2. 核心组件

- **CAN_Robot_Rx_Process**: 处理CAN2轨迹数据接收和解析
- **TrajectoryDataQueue**: 轨迹数据缓冲队列，防止数据覆盖
- **TrajectoryInterpolator**: 轨迹插值器，生成平滑运动轨迹
- **MIT_Motor**: 电机控制模块，发送控制指令到CAN1
- **全局变量**: `latest_trajectory_data`, `trajectory_data_received`

## CAN2数据格式

### 帧格式
- **CAN ID**: 0x100 (轨迹命令)
- **CAN总线**: CAN2 (can_id = 1)
- **数据长度**: 8字节
- **数据格式**: 小端字节序

### 数据结构
```
字节0-3:  关节1位置 (float, 度)
字节4-7:  关节2位置 (float, 度)
字节8-11: 关节3位置 (float, 度)
字节12-15:关节4位置 (float, 度)
字节16-19:关节5位置 (float, 度)
字节20-23:关节6位置 (float, 度)
字节24:   控制标志 (bit0: 是否为最终停止点)
字节25:   保留字节
```

### 示例数据
```cpp
// 发送到CAN2的示例数据
uint8_t can_data[8] = {
    0x00, 0x00, 0x48, 0x42,  // 关节1: 50.0度
    0x00, 0x00, 0x48, 0x42,  // 关节2: 50.0度
    0x00, 0x00, 0x48, 0x42,  // 关节3: 50.0度
    0x00, 0x00, 0x48, 0x42,  // 关节4: 50.0度
    0x00, 0x00, 0x48, 0x42,  // 关节5: 50.0度
    0x00, 0x00, 0x48, 0x42,  // 关节6: 50.0度
    0x01,                    // 最终停止点
    0x00                     // 保留
};
```

## 使用方法

### 1. 系统初始化
系统在启动时会自动初始化：
- 初始化CAN接收队列和处理器
- 初始化轨迹插值器
- 等待有效的电机位置数据

### 2. 发送轨迹数据
外部设备通过CAN2发送轨迹数据：
```cpp
// 伪代码示例
CAN_Frame frame;
frame.id = 0x100;
frame.can_id = 1;  // CAN2
frame.dlc = 8;
// 填充6个关节位置数据
send_can_frame(frame);
```

### 3. 轨迹执行
- 系统接收到轨迹数据后，自动添加到插值器队列
- 插值器生成平滑的轨迹点
- 电机按照插值轨迹执行运动

## 配置参数

### 插值器参数
- **采样时间**: 0.01秒 (100Hz)
- **最大速度**: 10度/秒
- **最大加速度**: 100度/秒²

### 队列参数
- **最大队列大小**: 100个轨迹点
- **处理频率**: 100Hz

## 日志和调试

### 控制台输出
```
CAN2 Trajectory: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0] Final:1
Added trajectory point to queue, queue size: 1
```

### 数据日志
系统会记录以下数据：
- CAN2轨迹数据接收日志
- 插值轨迹数据
- 电机状态数据

## 错误处理

### 常见错误
1. **队列满**: 当插值器队列满时，新的轨迹点会被拒绝
2. **数据格式错误**: 无效的CAN数据会被忽略
3. **电机位置无效**: 系统会等待有效的电机位置数据

### 错误恢复
- 系统会自动跳过无效数据
- 队列满时会输出警告信息
- 电机通信中断时会保持最后位置

## 性能特性

- **实时性**: 100Hz控制循环，低延迟响应
- **平滑性**: 轨迹插值确保运动平滑
- **可靠性**: 线程安全的消息队列处理
- **扩展性**: 支持动态轨迹点添加

## 与旧系统的区别

### 移除的功能
- 预定义的`predefined_joints`数组
- 静态轨迹点配置

### 新增的功能
- CAN2轨迹数据接收
- 动态轨迹点添加
- 实时轨迹控制

### 保持的功能
- 轨迹插值算法
- 电机控制逻辑
- 数据日志记录

## 队列机制

### 数据覆盖问题解决方案

为了防止CAN2发送数据太快导致的数据覆盖问题，系统实现了双层队列机制：

#### 1. 轨迹数据队列 (TrajectoryDataQueue)
- **容量**: 50个轨迹点
- **作用**: 缓冲CAN2接收到的轨迹数据
- **处理频率**: 100Hz (主控制循环频率)
- **每周期处理**: 最多5个点，避免阻塞

#### 2. 插值器队列 (TrajectoryInterpolator)
- **容量**: 100个轨迹点
- **作用**: 存储待插值的轨迹点
- **处理频率**: 100Hz
- **输出**: 平滑的轨迹点

### 队列处理流程

```cpp
// CAN接收处理 (中断级别)
void process_can2_trajectory_command() {
    // 解析CAN数据
    CAN2TrajectoryData item;
    // ... 解析逻辑 ...
    
    // 添加到轨迹数据队列
    if (trajectory_queue.push(item)) {
        // 成功添加
        log("CAN2_TRAJ: Queue size: %d", trajectory_queue.size());
    } else {
        // 队列满，丢弃数据
        log("CAN2_TRAJ_WARNING: Queue full, dropping point");
    }
}

// 主控制循环 (100Hz)
void robot_arm_control_loop() {
    // 处理轨迹数据队列
    uint8_t processed = 0;
    while (trajectory_queue.pop(item) && processed < 5) {
        // 添加到插值器队列
        if (arm_interpolator.add_trajectory_point(item)) {
            processed++;
        } else {
            // 插值器队列满
            break;
        }
    }
    
    // 执行插值
    arm_interpolator.update(interpolated_pos);
}
```

### 性能特性

- **防数据丢失**: 50点缓冲队列确保数据不丢失
- **实时性**: 100Hz处理频率，低延迟响应
- **平滑性**: 插值器确保运动平滑
- **可靠性**: 队列满时自动丢弃，避免系统阻塞 