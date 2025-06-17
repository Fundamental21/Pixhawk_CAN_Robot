#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "../CAN_Robot_Tx/CAN_Robot_Common.h"

// C++ style constants
namespace CANRobotConfig {
    constexpr uint16_t DENSE_QUEUE_SIZE = 1000;
    constexpr float INTERPOLATION_PERIOD = 0.001f; // 1ms for interpolation
    constexpr float MAX_COMMAND_DELAY = 0.5f;      // 500ms max delay
    constexpr float POSITION_PRECISION = 0.01f;    // 0.01 degree precision
}

// C++ style dense position queue
struct DensePositionQueue {
    float data[CANRobotConfig::DENSE_QUEUE_SIZE]{};
    uint16_t head{0};
    uint16_t tail{0};
    uint16_t count{0};
    
    DensePositionQueue() = default;
    
    // C++ style member functions
    bool is_full() const noexcept {
        return count >= CANRobotConfig::DENSE_QUEUE_SIZE;
    }
    
    bool is_empty() const noexcept {
        return count == 0;
    }
    
    uint16_t size() const noexcept {
        return count;
    }
    
    void clear() noexcept {
        head = 0;
        tail = 0;
        count = 0;
    }
};

// C++ style motor joint structure with RAII
struct MotorJoint {
    uint8_t can_id{0};
    uint8_t motor_id{0};
    MotorType type{MotorType::MIT};
    
    // Motion planning state
    float current_position{0.0f};
    float target_position{0.0f};
    float current_velocity{0.0f};
    float max_velocity{10.0f};
    float max_acceleration{20.0f};
    
    // Interpolation state
    bool is_moving{false};
    float start_position{0.0f};
    float motion_duration{0.0f};
    float elapsed_time{0.0f};
    
    // Dense position queue for smooth interpolation
    DensePositionQueue position_queue;
    
    MotorJoint() = default;
    
    // C++ style member functions
    void reset() noexcept {
        current_position = 0.0f;
        target_position = 0.0f;
        current_velocity = 0.0f;
        is_moving = false;
        start_position = 0.0f;
        motion_duration = 0.0f;
        elapsed_time = 0.0f;
        position_queue.clear();
    }
    
    bool has_reached_target() const noexcept {
        return fabsf(current_position - target_position) < CANRobotConfig::POSITION_PRECISION;
    }
};

// Main interpolation class with RAII and modern C++
class CAN_Robot_Interpolation {
public:
    // Singleton pattern with C++ style
    static CAN_Robot_Interpolation& get_instance() {
        static CAN_Robot_Interpolation instance;
        return instance;
    }
    
    // Core interface functions
    bool init() noexcept;
    void set_joint_target(uint8_t joint_id, float position) noexcept;
    void set_all_joints_target(const JointAngles& angles) noexcept;
    void update_interpolation(float dt) noexcept;
    bool execute_interpolation() noexcept;
    
    // Status getters (const qualified)
    float get_joint_position(uint8_t joint_id) const noexcept;
    JointAngles get_all_positions() const noexcept;
    bool is_motion_complete() const noexcept;
    
    // Queue management
    bool enqueue_position(uint8_t joint_id, float position) noexcept;
    bool dequeue_position(uint8_t joint_id, float& position) noexcept;
    void clear_all_queues() noexcept;
    
    // Configuration
    void set_joint_limits(uint8_t joint_id, float max_vel, float max_accel) noexcept;

private:
    MotorJoint _joints[JOINT_MOTOR_COUNT];
    bool _initialized{false};
    uint32_t _last_update_time_ms{0};
    
    // Private constructor for singleton
    CAN_Robot_Interpolation() = default;
    
    // Delete copy operations for singleton
    CAN_Robot_Interpolation(const CAN_Robot_Interpolation&) = delete;
    CAN_Robot_Interpolation& operator=(const CAN_Robot_Interpolation&) = delete;
    
    // Private helper functions
    void plan_trapezoidal_motion(MotorJoint& joint) noexcept;
    float calculate_trapezoidal_position(const MotorJoint& joint, float time) const noexcept;
    bool send_motor_command(const MotorJoint& joint, float position) const noexcept;
}; 