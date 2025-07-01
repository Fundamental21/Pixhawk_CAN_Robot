# KEGU夹爪电机CAN通信协议实现

## 协议概述

KEGU夹爪电机使用CAN标准帧进行通信，电机ID为7。以下是具体的通信协议实现：

## 1. 总线启动指令 (初始化)

**用途**: 系统启动时发送，初始化CAN总线上的KEGU电机

**CAN帧格式**:
```
CAN ID: 0x000
DLC: 8
Data: [0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
       ↑     ↑
       启动  电机ID(7)
```

**实现位置**: `create_kegu_motor_frame()` 中的 `KEGU_CMD_INIT` 分支

## 2. 电机使能和模式设置指令

### 2.1 电流模式使能

**用途**: 使能电机电流控制模式

**CAN帧格式**:
```
CAN ID: 0x207 (0x200 + 电机ID)
DLC: 8
Data: [0x00, 0x10, 0x0f, 0x00, 0xB8, 0x0B, 0x00, 0x00]
       ↑     ↑     ↑     ↑     ←─────┴─────────────→
       预留  电流   预留  预留  最大速度3000RPM
            模式
```

**说明**:
- `0x10` = 电流控制模式
- `0x0BB8` = 3000 (RPM，小端格式)

### 2.2 速度模式使能

**用途**: 使能电机速度控制模式

**CAN帧格式**:
```
CAN ID: 0x207 (0x200 + 电机ID)
DLC: 8
Data: [0x00, 0x03, 0x0f, 0x00, 0xB8, 0x0B, 0x00, 0x00]
       ↑     ↑     ↑     ↑     ←─────┴─────────────→
       预留  速度   预留  预留  最大速度3000RPM
            模式
```

**说明**:
- `0x03` = 速度控制模式
- `0x0BB8` = 3000 (RPM，小端格式)

## 3. 控制指令

### 3.1 电流控制指令

**用途**: 设置电机输出电流

**CAN帧格式**:
```
CAN ID: 0x407 (0x400 + 电机ID)
DLC: 8
Data: [0x00, 0x00, 0x00, 0x00, 电流L, 电流H, 0x00, 0x00]
       ←─────────────速度─────────→ ←───电流───→
```

**说明**:
- Byte0-3: 速度值 (设为0，因为是电流控制)
- Byte4-5: 电流值，单位10mA，小端格式
- 例如: 2.5A = 250个10mA = 0x00FA

**代码实现**:
```cpp
int16_t current_10ma = static_cast<int16_t>(value * 100.0f);  // 安培转换为10mA
frame.data[4] = static_cast<uint8_t>(current_10ma & 0xFF);
frame.data[5] = static_cast<uint8_t>((current_10ma >> 8) & 0xFF);
```

### 3.2 速度控制指令

**用途**: 设置电机转速

**CAN帧格式**:
```
CAN ID: 0x407 (0x400 + 电机ID)
DLC: 8
Data: [速度B0, 速度B1, 速度B2, 速度B3, 0x00, 0x00, 0x00, 0x00]
       ←─────────────速度(RPM)─────────→
```

**说明**:
- Byte0-3: 速度值，单位RPM，小端格式
- Byte4-7: 保留，填0
- 例如: 100 RPM = 0x00000064

**代码实现**:
```cpp
int32_t speed_rpm = static_cast<int32_t>(value);
frame.data[0] = static_cast<uint8_t>(speed_rpm & 0xFF);
frame.data[1] = static_cast<uint8_t>((speed_rpm >> 8) & 0xFF);
frame.data[2] = static_cast<uint8_t>((speed_rpm >> 16) & 0xFF);
frame.data[3] = static_cast<uint8_t>((speed_rpm >> 24) & 0xFF);
```

## 控制流程

### 电流控制流程
1. 发送总线启动指令 (`KEGU_CMD_INIT`)
2. 发送电流模式使能指令 (`KEGU_CMD_ENABLE_CUR`)  
3. 发送电流设置指令 (`KEGU_CMD_SET_CUR`)

### 速度控制流程
1. 发送总线启动指令 (`KEGU_CMD_INIT`)
2. 发送速度模式使能指令 (`KEGU_CMD_ENABLE_POS`)
3. 发送速度设置指令 (`KEGU_CMD_POSITION`)

## 函数映射

| 控制模式 | MotorControlMode | KEGU命令 | 函数 |
|---------|------------------|----------|------|
| 初始化 | `CTRL_MODE_INIT` | `KEGU_CMD_INIT` | `robot_arm_init()` |
| 使能电流 | `CTRL_MODE_ENABLE_CUR` | `KEGU_CMD_ENABLE_CUR` | `set_gripper_current()` |
| 使能速度 | `CTRL_MODE_ENABLE_POS` | `KEGU_CMD_ENABLE_POS` | `set_gripper_velocity()` |
| 设置电流 | `CTRL_MODE_CURRENT` | `KEGU_CMD_SET_CUR` | `set_gripper_current()` |
| 设置速度 | `CTRL_MODE_VELOCITY` | `KEGU_CMD_POSITION` | `set_gripper_velocity()` |

## 使用示例

```cpp
// 初始化 (在robot_arm_init中自动调用)
// CAN ID: 0x000, Data: [0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

// 电流控制
rover.set_gripper_current(2.5f);  // 设置2.5A电流
// 1. CAN ID: 0x207, Data: [0x00, 0x10, 0x0f, 0x00, 0xB8, 0x0B, 0x00, 0x00] (使能电流模式)
// 2. CAN ID: 0x407, Data: [0x00, 0x00, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00] (设置250个10mA)

// 速度控制  
rover.set_gripper_velocity(100.0f);  // 设置100 RPM
// 1. CAN ID: 0x207, Data: [0x00, 0x03, 0x0f, 0x00, 0xB8, 0x0B, 0x00, 0x00] (使能速度模式)
// 2. CAN ID: 0x407, Data: [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] (设置100 RPM)
```

## 注意事项

1. **字节序**: 所有多字节数据使用小端格式
2. **单位转换**: 
   - 电流: 输入安培，转换为10mA单位
   - 速度: 输入度/秒，在应用层转换为RPM
3. **模式切换**: 每次改变控制模式都需要先发送使能指令
4. **CAN ID**: 严格按照 `0x200+ID` 和 `0x400+ID` 的格式
5. **数据长度**: 固定8字节，不足部分填0 