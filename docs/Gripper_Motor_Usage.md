# 夹爪电机控制功能使用指南

## 概述

新增的夹爪电机功能为现有的6个关节电机系统添加了第7个电机（夹爪电机），具有以下特点：

- **电机类型**: KEGU电机
- **电机ID**: 7 (CAN总线1)
- **控制模式**: 电流控制和速度控制
- **数据获取**: 自动上报位置、速度、电流数据，无需主动请求
- **与关节电机的区别**: 夹爪电机不使用位置控制，主要用于力控制

## 控制接口

### 基础控制函数

#### 1. 电流控制
```cpp
// 设置夹爪电机电流值 (单位: 安培)
// 正值 = 夹紧，负值 = 松开
rover.set_gripper_current(2.5f);   // 2.5A 夹紧
rover.set_gripper_current(-1.0f);  // 1.0A 松开
rover.set_gripper_current(0.0f);   // 停止
```

#### 2. 速度控制
```cpp
// 设置夹爪电机速度值 (单位: 度/秒)
// 正值 = 夹紧方向，负值 = 松开方向
rover.set_gripper_velocity(50.0f);   // 50度/秒 夹紧
rover.set_gripper_velocity(-30.0f);  // 30度/秒 松开
rover.set_gripper_velocity(0.0f);    // 停止
```

### 便捷控制函数

#### 1. 夹爪张开
```cpp
// 按百分比速度张开夹爪 (0-100%)
rover.gripper_open(50.0f);  // 50% 速度张开
rover.gripper_open(100.0f); // 最大速度张开
```

#### 2. 夹爪夹紧
```cpp
// 按百分比速度夹紧夹爪 (0-100%)
rover.gripper_close(30.0f);  // 30% 速度夹紧
rover.gripper_close(80.0f);  // 80% 速度夹紧
```

#### 3. 夹爪停止
```cpp
// 立即停止夹爪运动
rover.gripper_stop();
```

### 状态获取函数

```cpp
// 获取夹爪当前位置 (度)
float position = rover.get_gripper_position();

// 获取夹爪当前速度 (度/秒)
float velocity = rover.get_gripper_velocity();

// 获取夹爪当前电流 (安培)
float current = rover.get_gripper_current();
```

## 使用示例

### 示例1: 基本的夹取操作
```cpp
// 1. 张开夹爪准备夹取
rover.gripper_open(50.0f);
hal.scheduler->delay(2000);  // 等待2秒

// 2. 夹紧物体
rover.gripper_close(30.0f);
hal.scheduler->delay(1000);  // 等待1秒

// 3. 保持夹紧力度
rover.set_gripper_current(1.5f);  // 维持1.5A夹紧力

// 4. 检查是否成功夹取
float current = rover.get_gripper_current();
if (current > 1.0f) {
    // 夹取成功
    hal.console->printf("Object gripped successfully, current: %.2f A\n", current);
} else {
    // 夹取失败
    hal.console->printf("Failed to grip object\n");
}
```

### 示例2: 精确力控制
```cpp
// 渐进式夹紧控制
for (float force = 0.5f; force <= 3.0f; force += 0.5f) {
    rover.set_gripper_current(force);
    hal.scheduler->delay(500);  // 每0.5秒增加力度
    
    float actual_current = rover.get_gripper_current();
    hal.console->printf("Target: %.1fA, Actual: %.2fA\n", force, actual_current);
    
    // 如果检测到阻力，停止增加力度
    if (actual_current >= force * 0.9f) {
        hal.console->printf("Object detected at %.2fA\n", actual_current);
        break;
    }
}
```

### 示例3: 在机器人控制循环中集成
```cpp
// 在 robot_arm_control_loop() 中添加夹爪控制
void Rover::robot_arm_control_loop() {
    // ... 现有的关节电机控制代码 ...
    
    // 夹爪状态监控
    gripper_position = get_gripper_position();
    gripper_velocity = get_gripper_velocity();
    gripper_current = get_gripper_current();
    
    // 根据任务状态控制夹爪
    static uint32_t last_gripper_cmd_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    
    if (now_ms - last_gripper_cmd_ms > 100) {  // 每100ms检查一次
        if (/* 需要夹取物体的条件 */) {
            gripper_close(50.0f);
        } else if (/* 需要释放物体的条件 */) {
            gripper_open(50.0f);
        }
        last_gripper_cmd_ms = now_ms;
    }
}
```

## 技术特点

### 1. 数据流向
- **发送**: 夹爪电机接收电流/速度控制指令
- **接收**: 夹爪电机自动上报位置、速度、电流数据（无需GET请求）

### 2. 安全限制
- **电流限制**: ±5.0A
- **速度限制**: ±100.0 度/秒
- **百分比转换**: 便捷函数中100% = 5.0A最大电流

### 3. 日志记录
夹爪电机的数据会以特殊格式记录在日志中：
```
GRIPPER_RX: CAN1 ID:0x07 Spd:12.30 Curr:2100 Pos:4520
```

注意：KEGU电机的数据格式与MIT电机不同，电流单位为mA，位置可能为脉冲数。

## 注意事项

1. **初始化**: 夹爪电机在 `robot_arm_init()` 中自动初始化
2. **线程安全**: 所有函数都是线程安全的，通过CAN队列系统处理
3. **实时性**: 控制指令通过专用的CAN发送线程处理，保证实时性
4. **故障检测**: 可通过电流反馈检测夹取是否成功
5. **与关节电机兼容**: 夹爪电机功能不影响现有6个关节电机的操作

## 调试和测试

### 启用调试输出
在编译时定义 `ARDUPILOT_BUILD` 可以启用详细的调试输出：
```
Gripper Motor (KEGU): CAN_ID=0, Motor_ID=7, Mode=CURRENT
Gripper Current Set: 2.50 A
Gripper Velocity Set: 50.00 deg/s
```

### CAN总线监控
可以通过日志查看夹爪电机的CAN通信：
- 发送日志: `CAN TX AUX` 开头的消息
- 接收日志: `GRIPPER_RX` 开头的消息

这样就完成了夹爪电机的完整功能实现！ 