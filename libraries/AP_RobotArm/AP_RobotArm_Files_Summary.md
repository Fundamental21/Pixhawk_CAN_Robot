# AP_RobotArm 文件功能总结

本文档总结了 ArduPilot Rover 中 `AP_RobotArm` 库的各个 C++ 源文件的核心功能和作用。

## 文件结构概览

```
AP_RobotArm/
├── AP_ChassisTask.cpp         # 机械臂轨迹任务管理
├── AP_CAN_Processor.cpp       # CAN消息处理与解析
├── AP_KinematicsLibrary.cpp   # 机械臂正逆运动学计算
└── AP_MIT_Motor.cpp           # MIT电机CAN通信控制
```

---

## 1. AP_ChassisTask.cpp - 机械臂轨迹任务管理

### 核心功能
- **预定义轨迹生成**：提供50个预设关节角度序列，用于机械臂运动规划
- **ArduPilot兼容性**：替代原STM32 HAL代码，提供ArduPilot环境下的任务接口
- **位置生成器**：根据电机ID返回对应的关节目标角度

### 主要特性
- 支持6自由度机械臂（6个关节）
- 循环轨迹播放功能
- USART兼容性接口（ArduPilot模式下为空实现）
- 集成GCS消息发送功能

### 关键函数
```cpp
float generate_position(MIT_Motor::MotorInstance* motor);  // 位置生成
void chassis_task(void const* pvParameters);              // 主任务函数
void usart1_init(uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_num);  // 串口初始化
```

---

## 2. AP_CAN_Processor.cpp - CAN消息处理与解析

### 核心功能
- **高效CAN消息队列处理**：使用ArduPilot的`ObjectBuffer`实现线程安全的消息队列
- **实时数据解析**：支持100Hz频率的电机数据解算（位置、速度、电流、温度）
- **双总线支持**：区分CAN1和CAN2总线，支持多达16个电机（每总线8个）
- **TihuMotor控制器**：提供完整的MIT电机控制命令接口

### 主要特性
- **内存对齐优化**：使用32字节对齐的数据结构，提升缓存性能
- **实时性保证**：每次处理限制在500μs内，确保系统实时性
- **统计监控**：详细的处理统计信息，包括成功率、错误率等
- **类型安全**：严格的数据类型检查和边界验证

### 核心数据结构
```cpp
// 全局电机数据实例
MitVelocityData mit_receive_velocity;      // 速度数据
MitPositionData mit_receive_position;      // 位置数据  
MitCurrentData mit_receive_current;        // 电流数据
MitTemperatureData mit_receive_temperature; // 温度数据
MitStateData mit_receive_state;            // 状态数据
```

### 关键功能
- **帧验证与解析**：支持多种命令类型（0x04电流、0x06速度、0x08位置等）
- **比例因子转换**：自动应用适当的单位转换
- **错误处理**：完善的异常情况处理和日志记录

---

## 3. AP_KinematicsLibrary.cpp - 机械臂正逆运动学计算

### 核心功能
- **正运动学计算**：从关节角度计算末端执行器位置和姿态
- **逆运动学求解**：从目标位置和姿态计算关节角度
- **多解优化**：从多个逆运动学解中选择最优解
- **DH参数支持**：基于Denavit-Hartenberg参数的运动学建模

### 主要特性
- **双臂配置支持**：支持左臂(L)和右臂(R)配置
- **关节限位检查**：验证解的有效性，确保在关节限制范围内
- **欧拉角转换**：支持ZYX和ZYZ欧拉角序列
- **成本函数优化**：基于权重的解选择算法

### 核心算法
```cpp
// 正运动学：关节角度 → 末端位置/姿态
void forward_kinematic(const RobotArmConfig& config, 
                      const JointAngles& q, 
                      Pose& result);

// 逆运动学：目标位置/姿态 → 关节角度
bool inverse_kinematic(const RobotArmConfig& config, 
                      const Pose& target,
                      const JointAngles& nominal, 
                      IKSolutions& solutions);
```

### 机械臂参数
- **6自由度关节**：每个关节有特定的运动范围限制
- **几何参数**：s1=875, s2=±162.5, s3=210, s4=260, s5=410（单位：mm）
- **DH参数表**：完整的7个变换矩阵定义

---

## 4. AP_MIT_Motor.cpp - MIT电机CAN通信控制

### 核心功能
- **双重发送策略**：优先使用DroneCAN，备用HAL直接发送
- **多种控制模式**：支持位置、速度、电流三种控制模式
- **状态查询**：实时获取电机位置、速度、电流、温度信息
- **错误处理与统计**：完整的发送统计和错误跟踪

### 主要特性
- **ArduPilot集成**：使用ArduPilot的CAN管理器和DroneCAN协议
- **实时日志记录**：详细的CAN消息发送日志
- **多接口支持**：支持多个CAN接口的自动切换
- **测试功能**：内置CAN发送测试功能

### 控制命令
```cpp
// 电机控制命令
void tihu_motor_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_type, float target_value);

// 单字节控制命令  
void tihu_motor_one_byte_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t cmd);

// CAN帧发送（双重策略）
bool write_frame(AP_HAL::CANFrame& frame, uint64_t timeout_us);
```

### 统计监控
```cpp
struct CANStats {
    uint32_t total_frames;        // 总发送帧数
    uint32_t dronecan_success;    // DroneCAN成功次数
    uint32_t dronecan_failures;   // DroneCAN失败次数  
    uint32_t hal_success;         // HAL成功次数
    uint32_t hal_failures;        // HAL失败次数
};
```

---

## 系统集成架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  AP_ChassisTask │───▶│ AP_MIT_Motor    │───▶│ CAN Bus Network │
│  (轨迹生成)      │    │ (电机控制)       │    │ (物理总线)       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │                       │
┌─────────────────┐    ┌─────────────────┐              │
│AP_KinematicsLib │    │AP_CAN_Processor │◀─────────────┘
│ (运动学计算)     │    │ (消息解析)       │
└─────────────────┘    └─────────────────┘
```

## 数据流向

1. **控制流**：`AP_ChassisTask` → `AP_MIT_Motor` → `CAN总线`
2. **反馈流**：`CAN总线` → `AP_CAN_Processor` → 全局数据结构
3. **计算流**：`AP_KinematicsLibrary` ↔ 控制系统

## 性能特征

- **实时性**：支持100Hz控制频率
- **可靠性**：多重错误检查和恢复机制  
- **可扩展性**：支持多达16个电机的同时控制
- **兼容性**：完全集成ArduPilot生态系统

## 配置要求

- ArduPilot CAN接口已启用
- DroneCAN协议支持
- 足够的内存用于消息队列（建议200帧缓冲）
- 实时调度器支持 