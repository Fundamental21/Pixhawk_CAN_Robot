#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_CAN_Processor.h"
#include "AP_MIT_Motor.h"

namespace RobotArm {

class DHParameters {
public:
    DHParameters() : alpha(0.0f), a(0.0f), d(0.0f), theta_offset(0.0f) {}
    DHParameters(float alpha_param, float a_param, float d_param, float theta_offset_param)
        : alpha(alpha_param), a(a_param), d(d_param), theta_offset(theta_offset_param) {}

    float alpha;
    float a;
    float d;
    float theta_offset;
};

class RobotArmConfig {
public:
    RobotArmConfig() = default;
    ~RobotArmConfig() = default;

    std::array<DHParameters, 7> dh;
    std::array<std::array<float, 2>, 6> joint_limits;
    std::array<float, 5> geometric_params;
};

class Pose {
public:
    Pose() : position{0.0f, 0.0f, 0.0f}, orientation{0.0f, 0.0f, 0.0f} {}
    
    std::array<float, 3> position;
    std::array<float, 3> orientation;
};

class JointAngles {
public:
    JointAngles() : angles{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {}
    
    std::array<float, 6> angles;
};

class IKSolutions {
public:
    IKSolutions() : count(0) {}
    
    std::array<JointAngles, 96> solutions;
    int count;
};

// Function declarations - using const char* instead of std::string for embedded compatibility
void init_robot_arm_config(RobotArmConfig& config, const char* arm_mode);

void forward_kinematic(const RobotArmConfig& config, const JointAngles& q, Pose& result);

bool inverse_kinematic(const RobotArmConfig& config, const Pose& target, 
                      const JointAngles& nominal, IKSolutions& solutions);

bool find_optimal_solution(const RobotArmConfig& config, const IKSolutions& solutions,
                         const JointAngles& nominal, const std::array<float, 6>& weights,
                         JointAngles& optimal);

} // namespace RobotArm 