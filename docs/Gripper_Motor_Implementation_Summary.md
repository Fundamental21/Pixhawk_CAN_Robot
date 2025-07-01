# KEGU夹爪电机(ID=7)功能实现总结

## 已完成的修改

### 1. 电机初始化 (`Rover/Rover.cpp`)
- 添加了夹爪电机实例: `&motor_instances[0][6]` (对应ID=7)
- 设置电机类型为 `MOTOR_TYPE_KEGU`
- 初始化模式为 `CTRL_MODE_INIT`
- 添加夹爪电机状态数据获取

### 2. 状态变量声明 (`Rover/Rover.h`)
- 添加夹爪电机状态变量:
  - `float gripper_position`
  - `float gripper_velocity` 
  - `float gripper_current`
- 添加公共接口函数声明

### 3. CAN接收处理 (`libraries/CAN_Robot_Rx/CAN_Robot_Rx_Process.cpp`)
- 修改 `is_kegu_motor_id()` 函数，添加对ID=7的识别
- 在KEGU电机反馈处理中添加夹爪电机特殊日志记录
- 夹爪电机数据自动上报，无需GET请求

### 4. CAN发送处理 (`libraries/AP_DroneCAN/AP_DroneCAN.cpp`)
- 在 `robot_can_tx_loop()` 中添加夹爪电机命令处理
- 夹爪电机不发送GET请求，只处理控制命令
- 添加专门的调试日志

### 5. 电机控制函数 (`Rover/MIT_Motor.cpp`)
- 实现KEGU夹爪电机控制函数:
  - `set_gripper_current()` - 电流控制
  - `set_gripper_velocity()` - 速度控制
  - `get_gripper_position()` - 位置获取
  - `get_gripper_velocity()` - 速度获取
  - `get_gripper_current()` - 电流获取
- 添加便捷控制函数:
  - `gripper_open()` - 张开夹爪
  - `gripper_close()` - 夹紧夹爪
  - `gripper_stop()` - 停止夹爪
- KEGU电机需要先发送ENABLE命令

### 6. 接口声明 (`Rover/MIT_Motor.h`)
- 在MIT_Motor命名空间中添加夹爪控制函数声明

### 7. 公共接口实现 (`Rover/Rover.cpp`)
- 实现Rover类的夹爪控制公共接口函数
- 这些函数调用MIT_Motor命名空间中的相应函数

## 技术特点

### KEGU电机 vs MIT电机的区别
1. **初始化**: KEGU电机需要先发送INIT和ENABLE命令
2. **CAN协议**: 
   - 初始化: CAN ID = 0x000
   - 使能: CAN ID = 0x200 + 电机ID  
   - 控制: CAN ID = 0x400 + 电机ID
3. **数据格式**: 
   - 电流单位: 10mA (输入安培转换为10mA单位)
   - 速度单位: RPM
   - 位置单位: 脉冲数 (需要转换为度数)
4. **通信方式**: 自动上报数据，无需GET请求
5. **控制模式**: 支持电流和速度控制

### 安全特性
- 电流限制: ±5.0A
- 速度限制: ±100.0 度/秒
- 便捷函数百分比控制 (0-100%)

### 日志记录
- 发送: `CAN TX AUX` 格式
- 接收: `GRIPPER_RX` 特殊格式
- 调试: 详细的电机状态输出

## 使用方法

### 基本控制
```cpp
// 电流控制 (推荐用于力控制)
rover.set_gripper_current(2.0f);   // 2A夹紧
rover.set_gripper_current(-1.0f);  // 1A松开

// 速度控制
rover.set_gripper_velocity(50.0f);  // 50度/秒夹紧

// 便捷控制
rover.gripper_close(50.0f);  // 50%力度夹紧
rover.gripper_open(30.0f);   // 30%力度松开
rover.gripper_stop();        // 停止
```

### 状态获取
```cpp
float pos = rover.get_gripper_position();  // 度
float vel = rover.get_gripper_velocity();  // 度/秒  
float cur = rover.get_gripper_current();   // 安培
```

## 兼容性
- 与现有6个关节电机(MIT类型)完全兼容
- 使用相同的CAN总线(CAN1)
- 通过统一的队列系统处理
- 不影响现有功能

## 测试建议
1. 首先测试基本的电流控制
2. 验证状态数据是否正确接收
3. 测试便捷控制函数
4. 检查日志记录是否正常
5. 验证与关节电机的协调工作

这样就完成了KEGU夹爪电机(ID=7)的完整功能实现！ 