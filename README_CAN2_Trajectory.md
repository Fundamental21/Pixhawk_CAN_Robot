# CAN2轨迹数据接收系统使用指南

## 概述

本系统实现了从CAN2口接收外部轨迹数据的功能，替代了原来的预定义关节位置数组。现在可以通过CAN2总线实时发送6个关节的位置数据，系统会自动将其添加到轨迹插值器队列中，实现平滑的机器人臂运动控制。

## 主要变化

### 移除的功能
- ❌ 预定义的`predefined_joints`数组
- ❌ 静态轨迹点配置

### 新增的功能
- ✅ CAN2轨迹数据接收 (ID: 0x100)
- ✅ 动态轨迹点添加
- ✅ 实时轨迹控制
- ✅ 外部设备接口

## 系统架构

```
外部设备 (CAN2) 
    ↓ [CAN帧: ID=0x100, 数据=6个关节位置]
AP_DroneCAN::processRxFromInterface()
    ↓ [CAN_Robot_Rx_Queue]
CAN_Robot_Rx_Process::process_can2_trajectory_command()
    ↓ [更新全局变量]
Rover::robot_arm_control_loop()
    ↓ [添加到插值器队列]
TrajectoryInterpolator::add_trajectory_point()
    ↓ [插值生成平滑轨迹]
MIT_Motor::MotorControl_Handler()
    ↓ [CAN1发送控制指令]
6个MIT电机
```

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

## 使用方法

### 1. 编译和部署

确保代码已正确编译并部署到Pixhawk设备上。

### 2. 连接CAN2接口

将外部设备连接到Pixhawk的CAN2接口。

### 3. 发送轨迹数据

外部设备需要按照以下格式发送CAN帧：

```cpp
// C++示例代码
struct CANFrame {
    uint32_t id = 0x100;  // 轨迹命令ID
    uint8_t dlc = 8;      // 数据长度
    uint8_t data[8];      // 数据内容
};

void send_trajectory(float joint_positions[6], bool is_final) {
    CANFrame frame;
    
    // 填充6个关节位置 (小端格式)
    for (int i = 0; i < 6; i++) {
        memcpy(&frame.data[i * 4], &joint_positions[i], sizeof(float));
    }
    
    // 设置控制标志
    frame.data[24] = is_final ? 0x01 : 0x00;
    frame.data[25] = 0x00;  // 保留字节
    
    // 发送到CAN2
    send_can_frame(frame);
}
```

### 4. 测试工具

使用提供的Python测试工具：

```bash
# 安装依赖
sudo apt-get install python3-socketcan

# 运行测试
python3 Tools/test_can2_trajectory.py can1 1.0
```

## 配置参数

### 插值器参数
- **采样时间**: 0.01秒 (100Hz)
- **最大速度**: 10度/秒
- **最大加速度**: 100度/秒²

### 队列参数
- **最大队列大小**: 100个轨迹点
- **处理频率**: 100Hz

## 监控和调试

### 控制台输出
系统会在控制台输出以下信息：
```
CAN2 Trajectory: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0] Final:1
Added trajectory point to queue, queue size: 1
```

### 数据日志
系统会记录以下数据：
- CAN2轨迹数据接收日志
- 插值轨迹数据
- 电机状态数据

### 日志查看
```bash
# 查看实时日志
tail -f /var/log/syslog | grep "CAN2_TRAJ"

# 查看电机状态
tail -f /var/log/syslog | grep "MIT_RX"
```

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

## 示例应用

### 1. 简单位置控制
```cpp
// 发送到位置 [0, 45, 90, 135, 180, 225]
float positions[6] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f};
send_trajectory(positions, false);
```

### 2. 连续轨迹
```cpp
// 发送多个轨迹点
float pos1[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float pos2[6] = {45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f};
float pos3[6] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

send_trajectory(pos1, false);
send_trajectory(pos2, false);
send_trajectory(pos3, true);  // 最终停止点
```

### 3. 动态轨迹生成
```cpp
// 根据传感器数据生成轨迹
for (int i = 0; i < 6; i++) {
    positions[i] = calculate_target_position(i, sensor_data);
}
send_trajectory(positions, false);
```

## 故障排除

### 1. 无法接收CAN数据
- 检查CAN2接口连接
- 确认CAN ID为0x100
- 验证数据长度为8字节

### 2. 轨迹执行异常
- 检查关节位置范围 (-360° 到 +360°)
- 确认电机通信正常
- 查看控制台错误信息

### 3. 系统响应慢
- 检查CAN总线负载
- 确认插值器参数设置
- 监控队列大小

## 技术支持

如果遇到问题，请：
1. 查看控制台输出和日志
2. 检查CAN数据格式
3. 验证硬件连接
4. 参考系统文档

## 更新日志

### v1.0.0 (当前版本)
- ✅ 实现CAN2轨迹数据接收
- ✅ 移除预定义关节位置数组
- ✅ 添加动态轨迹点支持
- ✅ 提供测试工具
- ✅ 完善错误处理 