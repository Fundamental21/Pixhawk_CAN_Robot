#include "CAN_Robot_interpolation.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "../CAN_Robot_Tx/CAN_Robot_Tx_Process.h"
#include <cmath>
#include <algorithm>

// Initialize the interpolation system
bool CAN_Robot_Interpolation::init() noexcept
{
    if (_initialized) {
        return true;
    }
    
    // Initialize all joints with default parameters
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        _joints[i].can_id = 1; // Default CAN ID
        _joints[i].motor_id = i;
        _joints[i].type = MotorType::MIT;
        _joints[i].reset();
    }
    
    _last_update_time_ms = AP_HAL::millis();
    _initialized = true;
    
    return true;
}

// Set target position for a specific joint
void CAN_Robot_Interpolation::set_joint_target(uint8_t joint_id, float position) noexcept
{
    if (joint_id >= JOINT_MOTOR_COUNT) {
        return;
    }
    
    MotorJoint& joint = _joints[joint_id];
    joint.target_position = position;
    joint.start_position = joint.current_position;
    joint.elapsed_time = 0.0f;
    joint.is_moving = true;
    
    plan_trapezoidal_motion(joint);
}

// Set target positions for all joints
void CAN_Robot_Interpolation::set_all_joints_target(const JointAngles& angles) noexcept
{
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        set_joint_target(i, angles.angles[i]);
    }
}

// Update interpolation for all joints
void CAN_Robot_Interpolation::update_interpolation(float dt) noexcept
{
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        MotorJoint& joint = _joints[i];
        
        if (!joint.is_moving) {
            continue;
        }
        
        joint.elapsed_time += dt;
        
        // Calculate new position using trapezoidal motion profile
        float new_position = calculate_trapezoidal_position(joint, joint.elapsed_time);
        joint.current_position = new_position;
        
        // Check if motion is complete
        if (joint.elapsed_time >= joint.motion_duration || joint.has_reached_target()) {
            joint.current_position = joint.target_position;
            joint.is_moving = false;
        }
        
        // Send command to motor
        send_motor_command(joint, joint.current_position);
    }
}

// Execute interpolation (main update function)
bool CAN_Robot_Interpolation::execute_interpolation() noexcept
{
    if (!_initialized) {
        return false;
    }
    
    const uint32_t current_time_ms = AP_HAL::millis();
    const float dt = (current_time_ms - _last_update_time_ms) * 0.001f; // Convert to seconds
    
    if (dt >= CANRobotConfig::INTERPOLATION_PERIOD) {
        update_interpolation(dt);
        _last_update_time_ms = current_time_ms;
    }
    
    return true;
}

// Get current position of a joint
float CAN_Robot_Interpolation::get_joint_position(uint8_t joint_id) const noexcept
{
    if (joint_id >= JOINT_MOTOR_COUNT) {
        return 0.0f;
    }
    
    return _joints[joint_id].current_position;
}

// Get all joint positions
JointAngles CAN_Robot_Interpolation::get_all_positions() const noexcept
{
    JointAngles angles{};
    
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        angles.angles[i] = _joints[i].current_position;
    }
    
    return angles;
}

// Check if all motions are complete
bool CAN_Robot_Interpolation::is_motion_complete() const noexcept
{
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        if (_joints[i].is_moving) {
            return false;
        }
    }
    return true;
}

// Enqueue position for a joint
bool CAN_Robot_Interpolation::enqueue_position(uint8_t joint_id, float position) noexcept
{
    if (joint_id >= JOINT_MOTOR_COUNT) {
        return false;
    }
    
    DensePositionQueue& queue = _joints[joint_id].position_queue;
    
    if (queue.is_full()) {
        return false;
    }
    
    queue.data[queue.tail] = position;
    queue.tail = (queue.tail + 1) % CANRobotConfig::DENSE_QUEUE_SIZE;
    queue.count++;
    
    return true;
}

// Dequeue position for a joint
bool CAN_Robot_Interpolation::dequeue_position(uint8_t joint_id, float& position) noexcept
{
    if (joint_id >= JOINT_MOTOR_COUNT) {
        return false;
    }
    
    DensePositionQueue& queue = _joints[joint_id].position_queue;
    
    if (queue.is_empty()) {
        return false;
    }
    
    position = queue.data[queue.head];
    queue.head = (queue.head + 1) % CANRobotConfig::DENSE_QUEUE_SIZE;
    queue.count--;
    
    return true;
}

// Clear all position queues
void CAN_Robot_Interpolation::clear_all_queues() noexcept
{
    for (uint8_t i = 0; i < JOINT_MOTOR_COUNT; i++) {
        _joints[i].position_queue.clear();
    }
}

// Set joint motion limits
void CAN_Robot_Interpolation::set_joint_limits(uint8_t joint_id, float max_vel, float max_accel) noexcept
{
    if (joint_id >= JOINT_MOTOR_COUNT) {
        return;
    }
    
    _joints[joint_id].max_velocity = max_vel;
    _joints[joint_id].max_acceleration = max_accel;
}

// Plan trapezoidal motion profile
void CAN_Robot_Interpolation::plan_trapezoidal_motion(MotorJoint& joint) noexcept
{
    const float distance = joint.target_position - joint.start_position;
    const float abs_distance = fabsf(distance);
    
    if (abs_distance < CANRobotConfig::POSITION_PRECISION) {
        joint.motion_duration = 0.0f;
        joint.is_moving = false;
        return;
    }
    
    // const float sign = (distance > 0.0f) ? 1.0f : -1.0f; // Unused variable, commented out
    const float max_vel = joint.max_velocity;
    const float max_accel = joint.max_acceleration;
    
    // Time to reach max velocity
    const float t_accel = max_vel / max_accel;
    
    // Distance covered during acceleration and deceleration
    const float accel_distance = 0.5f * max_accel * t_accel * t_accel;
    
    if (2.0f * accel_distance >= abs_distance) {
        // Triangular profile (no constant velocity phase)
        const float t_half = sqrtf(abs_distance / max_accel);
        joint.motion_duration = 2.0f * t_half;
    } else {
        // Trapezoidal profile
        const float cruise_distance = abs_distance - 2.0f * accel_distance;
        const float t_cruise = cruise_distance / max_vel;
        joint.motion_duration = 2.0f * t_accel + t_cruise;
    }
}

// Calculate position using trapezoidal motion profile
float CAN_Robot_Interpolation::calculate_trapezoidal_position(const MotorJoint& joint, float time) const noexcept
{
    if (time <= 0.0f) {
        return joint.start_position;
    }
    
    if (time >= joint.motion_duration) {
        return joint.target_position;
    }
    
    const float distance = joint.target_position - joint.start_position;
    const float abs_distance = fabsf(distance);
    const float sign = (distance > 0.0f) ? 1.0f : -1.0f;
    
    const float max_vel = joint.max_velocity;
    const float max_accel = joint.max_acceleration;
    
    // Time to reach max velocity
    const float t_accel = max_vel / max_accel;
    
    // Distance covered during acceleration
    const float accel_distance = 0.5f * max_accel * t_accel * t_accel;
    
    float position_offset = 0.0f;
    
    if (2.0f * accel_distance >= abs_distance) {
        // Triangular profile
        const float t_half = joint.motion_duration * 0.5f;
        if (time <= t_half) {
            // Acceleration phase
            position_offset = 0.5f * max_accel * time * time;
        } else {
            // Deceleration phase
            const float t_decel = time - t_half;
            position_offset = accel_distance + max_vel * t_half * 0.5f - 0.5f * max_accel * t_decel * t_decel;
        }
    } else {
        // Trapezoidal profile
        if (time <= t_accel) {
            // Acceleration phase
            position_offset = 0.5f * max_accel * time * time;
        } else if (time <= joint.motion_duration - t_accel) {
            // Constant velocity phase
            const float t_cruise = time - t_accel;
            position_offset = accel_distance + max_vel * t_cruise;
        } else {
            // Deceleration phase
            const float t_decel = time - (joint.motion_duration - t_accel);
            position_offset = abs_distance - 0.5f * max_accel * (t_accel - t_decel) * (t_accel - t_decel);
        }
    }
    
    return joint.start_position + sign * position_offset;
}

// Send command to motor
bool CAN_Robot_Interpolation::send_motor_command(const MotorJoint& joint, float position) const noexcept
{
    CAN_Robot_Tx_Process* tx_process = CAN_Robot_Tx_Process::get_singleton();
    if (tx_process == nullptr) {
        return false;
    }
    
    // Create motor frame and send command through the processor
    CAN_Robot_Tx_Process::create_motor_frame(
        joint.can_id, 
        joint.motor_id, 
        joint.type, 
        MotorControlMode::POSITION, 
        position
    );
    
    // Frame will be handled by the CAN driver
    return true;
} 