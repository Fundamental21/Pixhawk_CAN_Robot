# AP_RobotArm Library - CAN通信与控制系统

本文档提供AP_RobotArm库的完整使用指南，包括CAN接口配置和消息转发系统的详细说明。

## CAN接口配置总览

### CAN1 - Robot Arm Control (Dedicated)
- **Purpose**: Exclusively used for AP_RobotArm module communication
- **RX Messages**: Motor feedback, sensor data from robot arm components
- **TX Messages**: Motor control commands, configuration commands
- **Processing**: All CAN1 messages are processed by `AP_CAN_Processor`
- **Frequency**: Configurable via parameters

### CAN2 - Reserved Interface
- **Purpose**: Reserved for future expansion
- **Status**: Interface available, logging enabled
- **Processing**: Basic logging only, ready for future modules

## 架构图

```
Hardware CAN1 ──→ CanardInterface ──→ AP_CAN_Processor ──→ MIT_Motor/Controllers
                      ↑                      ↓
                   rxmsg1               control_signals
                   
Hardware CAN2 ──→ CanardInterface ──→ [Reserved for future use]
                      ↑
                   rxmsg2 (logged only)
```

## 使用说明

1. Connect robot arm controllers to **CAN1 only**
2. Configure CAN parameters for CAN1
3. CAN2 remains available for other systems

## 参数配置

- `CAN_D1_UC_CAN_RT`: CAN1 processing rate
- `CAN_D1_UC_MOT_RT`: CAN1 motor command rate
- `CAN_D2_UC_*`: CAN2 parameters (reserved)

## 实现细节

- MIT Motor controllers send to CAN1 via `write_aux_frame_to_iface(frame, timeout, 0)`
- CAN_Processor routes all traffic through CAN1 interface
- DroneCAN driver_index=0 handles CAN1, driver_index=1 handles CAN2

---

# ArduPilot CAN消息转发系统 - 统一架构版本

## 概述

本系统实现了从 `Hall_Can_Backend::handle_frame()` 到 `AP_RobotArm` 的CAN消息转发，支持**CAN1和CAN2总线区分**，工作频率为100Hz，完全符合ArduPilot嵌入式飞控代码标准。

**重要更新**：已将 `AP_CAN_Receive.cpp/.h` 完全合并到 `AP_CAN_Processor.cpp/.h` 中，提供统一的CAN处理和电机控制接口。

## 主要特性

### 统一架构
- ✅ **单一文件包含所有CAN功能**
- ✅ **移除STM32 HAL依赖**
- ✅ **完全兼容ArduPilot架构**
- ✅ **集成电机控制接口**

### CAN总线区分
- ✅ **独立处理CAN1和CAN2消息**
- ✅ **保留CAN总线来源信息**
- ✅ **统一队列处理，支持混合总线消息**
- ✅ **详细的总线级别日志记录**

### 高频率支持
- ✅ **100Hz消息转发频率**
- ✅ **200帧队列缓冲区（2秒@100Hz）**
- ✅ **实时性能保证（<500μs处理时间）**
- ✅ **批量处理优化（每次最多20帧）**

### 嵌入式优化
- ✅ **内存效率（总计~3.6KB）**
- ✅ **非阻塞队列操作**
- ✅ **CPU时间限制**
- ✅ **统计信息监控**

## 统一架构图

```
CAN硬件接口
    ↓
DroneCAN::CanardInterface::processRx()
  ├─ CAN1 (i=0) → handle_frame(msg, 1)
  └─ CAN2 (i=1) → handle_frame(msg, 2)
    ↓
Hall_Can_Backend::handle_frame(frame, can_bus_id)
    ↓
CAN_Frame_Item{frame, timestamp, can_bus_id}
    ↓
ObjectBuffer<CAN_Frame_Item> (200帧容量)
    ↓
AP_CAN_Processor::update() (100Hz调用)
    ↓
process_can_frame() → getTargetField()
    ↓
全局数据结构更新:
├─ mit_receive_velocity[channel][motor]
├─ mit_receive_position[channel][motor]  
├─ mit_receive_current[channel][motor]
├─ mit_receive_temperature[channel][motor]
└─ mit_receive_state[channel][motor]
    ↓
TihuMotorController API
├─ motorCtrl()
├─ motorOneByteCtrl()
├─ getMotorPosition()
├─ getMotorVelocity()
├─ getMotorCurrent()
└─ getMotorTemperature()
```

## 关键改进

### 1. 架构统一
```cpp
// 现在只需要包含一个头文件
#include "AP_CAN_Processor.h"

// 所有功能都通过统一的接口访问
AP_CAN_Processor* processor = AP_CAN_Processor::get_singleton();
float position = TihuMotorController::getMotorPosition(1, 2); // CAN1, Motor2
```

### 2. 移除外部依赖
```cpp
// 移除了这些不兼容的依赖：
// #include "cmsis_os.h"           // CMSIS-OS依赖
// #include "main.h"               // STM32 HAL依赖  
// #include "bsp_rng.h"            // 板级支持包依赖
// extern CAN_HandleTypeDef hcan1; // STM32 HAL CAN句柄

// 现在使用ArduPilot标准接口：
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
```

### 3. 增强的CAN总线处理
```cpp
// 正确的CAN总线区分处理
bool AP_CAN_Processor::process_can_frame(const AP_HAL::CANFrame& frame, uint8_t can_bus_id) {
    // 自动映射到正确的通道
    const CanChannel channel = (can_bus_id == 1) ? CanChannel::CHANNEL_1 : CanChannel::CHANNEL_2;
    
    // 类型安全的数据访问
    float* target = getTargetField(can_id, channel, cfg.data_struct_ptr);
    if (target) {
        *target = value;
        return true;
    }
}
```

### 4. 完整的电机控制API
```cpp
namespace RobotArm {
    class TihuMotorController {
    public:
        // 电机控制
        static void motorCtrl(uint8_t can_id, uint8_t motor_id, MotorCtrlMode mode, float value);
        static void motorOneByteCtrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_cmd);
        
        // 数据获取
        static float getMotorPosition(uint8_t can_id, uint8_t motor_id);
        static float getMotorVelocity(uint8_t can_id, uint8_t motor_id);
        static float getMotorCurrent(uint8_t can_id, uint8_t motor_id);
        static float getMotorTemperature(uint8_t can_id, uint8_t motor_id);
        
        // 快速验证（constexpr）
        static constexpr bool isValidCanId(uint8_t can_id);
        static constexpr bool isValidMotorId(uint8_t motor_id);
    };
}
```

## 使用方法

### 1. 简化的初始化
```cpp
// 只需要初始化一个组件
AP_CAN_Processor* processor = AP_CAN_Processor::get_singleton();
Hall_Can_Backend* hall_backend = Hall_Can_Backend::get_singleton();

// 获取CAN队列并初始化处理器
ObjectBuffer<AP_CAN_Processor::CAN_Frame_Item>* queue = 
    hall_backend->get_robotarm_can_queue();
processor->init(queue);
```

### 2. 100Hz更新循环
```cpp
// 在主循环中以100Hz频率调用
void update_100hz() {
    AP_CAN_Processor* processor = AP_CAN_Processor::get_singleton();
    if (processor) {
        processor->update();  // 处理CAN1和CAN2的消息
    }
}
```

### 3. 电机控制
```cpp
using namespace RobotArm;

// 控制电机
TihuMotorController::motorCtrl(1, 3, MotorCtrlMode::SPEED, 100.0f);     // CAN1电机3设置速度
TihuMotorController::motorCtrl(2, 5, MotorCtrlMode::POSITION, 180.0f);  // CAN2电机5设置位置

// 单字节控制
TihuMotorController::motorOneByteCtrl(1, 2, 0x06);  // 获取电机2速度

// 获取电机数据
float pos1 = TihuMotorController::getMotorPosition(1, 1);    // CAN1电机1位置
float vel2 = TihuMotorController::getMotorVelocity(2, 3);    // CAN2电机3速度
float cur1 = TihuMotorController::getMotorCurrent(1, 4);     // CAN1电机4电流
float temp2 = TihuMotorController::getMotorTemperature(2, 7); // CAN2电机7温度
```

### 4. 直接数据访问（高性能）
```cpp
using namespace RobotArm;

// 直接访问全局数据结构（最快）
float velocity = *mit_receive_velocity.get_safe(CanChannel::CHANNEL_1, 0); // CAN1电机1
float position = *mit_receive_position.get_safe(CanChannel::CHANNEL_2, 4); // CAN2电机5

// 类型安全检查
if (TihuMotorController::isValidCanId(can_id) && 
    TihuMotorController::isValidMotorId(motor_id)) {
    // 进行操作
}
```

## 支持的CAN消息类型

| 命令码 | 数据类型 | 比例因子 | 目标结构 |
|--------|----------|----------|----------|
| 0x04   | 电流     | 1.0/1000.0 | mit_receive_current |
| 0x06   | 速度     | 0.6/101.0 | mit_receive_velocity |
| 0x08   | 位置     | 360.0/262144.0 | mit_receive_position |
| 0x14   | 状态     | 1.0 | mit_receive_state |
| 0x32   | 温度     | 1.0 | mit_receive_temperature |

## 文件结构

```
libraries/AP_RobotArm/
├── AP_CAN_Processor.h          # 统一头文件（包含所有定义）
├── AP_CAN_Processor.cpp        # 统一实现文件（包含所有功能）
└── README_CAN_Integration.md   # 本文档

// 已删除的文件：
// ├── AP_CAN_Receive.h         # 已合并到AP_CAN_Processor.h
// └── AP_CAN_Receive.cpp       # 已合并到AP_CAN_Processor.cpp
```

## 性能指标

### 内存使用
- **队列缓冲区**: 200 × 18字节 = 3.6KB
- **全局数据结构**: 5 × 2 × 8 × 4字节 = 320字节
- **统计结构**: ~32字节
- **总内存**: < 4KB

### 处理性能
- **最大处理时间**: 500μs per update
- **最大帧数**: 20帧 per update
- **队列容量**: 200帧 (2秒@100Hz)
- **溢出处理**: 非阻塞，统计记录

### 实时性保证
- ✅ 非阻塞队列操作
- ✅ CPU时间限制（500μs）
- ✅ 批量处理优化
- ✅ 优先级管理

## 调试和监控

### 1. 日志监控
```bash
# 查看CAN处理统计
mavproxy> module load log
mavproxy> log search "CAN.*Motor.*Processed"

# 查看队列状态
mavproxy> log search "RobotArm CAN.*queue"
```

### 2. 统计信息
```cpp
// 获取处理统计
const auto& stats = processor->get_statistics();
GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Processed: %u, Invalid: %u, Filtered: %u", 
              (unsigned)stats.processed_frames, 
              (unsigned)stats.invalid_frames, 
              (unsigned)stats.filtered_frames);
```

### 3. 性能监控
```cpp
// 监控处理时间
GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Max process time: %llu μs", 
              (unsigned long long)stats.max_process_time_us);
GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Update calls: %u", 
              (unsigned)stats.update_calls);
```

## 迁移指南

### 从旧版本迁移
```cpp
// 旧版本（已删除）
// #include "AP_CAN_Receive.h"
// HAL_CAN_RxFifo0MsgPendingCallback(); // 不再需要

// 新版本（统一）
#include "AP_CAN_Processor.h"
AP_CAN_Processor::get_singleton()->update(); // 在100Hz循环中调用
```

### API兼容性
```cpp
// 原有的TihuMotorController API完全保持兼容
TihuMotorController::motorCtrl(can_id, motor_id, mode, value);      // ✅ 兼容
TihuMotorController::getMotorPosition(can_id, motor_id);           // ✅ 兼容

// 新增的API
TihuMotorController::getMotorVelocity(can_id, motor_id);           // ✨ 新增
TihuMotorController::getMotorCurrent(can_id, motor_id);            // ✨ 新增
TihuMotorController::getMotorTemperature(can_id, motor_id);        // ✨ 新增
```

## 故障排除

### 常见问题

1. **队列溢出**
   - 症状: "CAN queue overflow" 警告
   - 解决: 检查100Hz更新循环是否正常运行

2. **无消息处理**
   - 症状: processed_frames = 0
   - 解决: 验证CAN硬件连接和DroneCAN配置

3. **编译错误**
   - 症状: 找不到AP_CAN_Receive.h
   - 解决: 更新#include为"AP_CAN_Processor.h"

### 配置建议

```cpp
// 针对高频率应用的配置
const uint8_t max_frames_per_call = 20;        // 可调整为10-30
const uint64_t max_process_time_us = 500;      // 可调整为300-1000
const uint32_t queue_capacity = 200;           // 可调整为100-400
```

## 总结

统一后的CAN消息转发系统提供了：

1. **简化的架构** - 单一文件包含所有功能
2. **完整的兼容性** - 与ArduPilot完全兼容
3. **强大的功能** - CAN处理 + 电机控制一体化
4. **高性能实时处理** - 100Hz频率，低延迟保证
5. **生产级可靠性** - 完善的错误处理和监控
6. **易于维护** - 统一的代码基础，清晰的接口

系统现在更加简洁、可靠，可以安全部署到生产环境中的ArduPilot飞控系统。 