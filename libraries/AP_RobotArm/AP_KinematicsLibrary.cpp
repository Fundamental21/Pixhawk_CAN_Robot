#include "AP_KinematicsLibrary.h"
#include <cstring>
#include <algorithm>
#include <limits>

namespace RobotArm {

//--------------utils--------------
static void matrix_multiply(const std::array<std::array<float, 4>, 4>& a, 
                          const std::array<std::array<float, 4>, 4>& b, 
                          std::array<std::array<float, 4>, 4>& result) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

static int R_to_Euler(const std::array<std::array<float, 3>, 3>& rotm, 
                     const char* sequence, 
                     std::array<float, 3>& eul) {
    if (strcmp(sequence, "ZYX") == 0) {
        if (rotm[2][0] < 1) {
            if (rotm[2][0] > -1) {
                eul[0] = atan2(rotm[1][0], rotm[0][0]); // theta_z
                eul[1] = asin(-rotm[2][0]);             // theta_y
                eul[2] = atan2(rotm[2][1], rotm[2][2]); // theta_x
            } else { // rotm[2][0] = -1
                eul[0] = -atan2(-rotm[1][2], rotm[1][1]);
                eul[1] = M_PI / 2;
                eul[2] = 0;
            }
        } else { // rotm[2][0] = 1
            eul[0] = atan2(-rotm[1][2], rotm[1][1]);
            eul[1] = -M_PI / 2;
            eul[2] = 0;
        }
        return 0;
    } else if (strcmp(sequence, "ZYZ") == 0) {
        if (rotm[2][2] < 1) {
            if (rotm[2][2] > -1) {
                eul[0] = atan2(rotm[1][2], rotm[0][2]);   // theta_z1
                eul[1] = acos(rotm[2][2]);                // theta_y
                eul[2] = atan2(rotm[2][1], -rotm[2][0]);  // theta_z2
            } else { // rotm[2][2] = -1
                eul[0] = -atan2(rotm[1][0], rotm[1][1]);
                eul[1] = M_PI;
                eul[2] = 0;
            }
        } else { // rotm[2][2] = 1
            eul[0] = atan2(rotm[1][0], rotm[1][1]);
            eul[1] = 0;
            eul[2] = 0;
        }
        return 0;
    }
    return -1;
}

static void Euler_to_R(const std::array<float, 3>& eul, 
                      const char* S, 
                      std::array<std::array<float, 3>, 3>& R) {
    std::array<float, 3> ct, st;
    for (int j = 0; j < 3; ++j) {
        ct[j] = cos(eul[j]);
        st[j] = sin(eul[j]);
    }

    if (strcmp(S, "ZYX") == 0) {
        R[0][0] = ct[1]*ct[0];
        R[0][1] = st[2]*st[1]*ct[0] - ct[2]*st[0];
        R[0][2] = ct[2]*st[1]*ct[0] + st[2]*st[0];

        R[1][0] = ct[1]*st[0];
        R[1][1] = st[2]*st[1]*st[0] + ct[2]*ct[0];
        R[1][2] = ct[2]*st[1]*st[0] - st[2]*ct[0];

        R[2][0] = -st[1];
        R[2][1] = st[2]*ct[1];
        R[2][2] = ct[2]*ct[1];
    }
    else if (strcmp(S, "XYZ") == 0) {
        R[0][0] = ct[1]*ct[2];
        R[0][1] = -ct[1]*st[2];
        R[0][2] = st[1];

        R[1][0] = ct[0]*st[2] + ct[2]*st[0]*st[1];
        R[1][1] = ct[0]*ct[2] - st[0]*st[1]*st[2];
        R[1][2] = -ct[1]*st[0];

        R[2][0] = st[0]*st[2] - ct[0]*ct[2]*st[1];
        R[2][1] = ct[2]*st[0] + ct[0]*st[1]*st[2];
        R[2][2] = ct[0]*ct[1];
    }
    else {
        R[0][0] = ct[0]*ct[2]*ct[1] - st[0]*st[2];
        R[0][1] = -ct[0]*ct[1]*st[2] - st[0]*ct[2];
        R[0][2] = ct[0]*st[1];

        R[1][0] = st[0]*ct[2]*ct[1] + ct[0]*st[2];
        R[1][1] = -st[0]*ct[1]*st[2] + ct[0]*ct[2];
        R[1][2] = st[0]*st[1];

        R[2][0] = -st[1]*ct[2];
        R[2][1] = st[1]*st[2];
        R[2][2] = ct[1];
    }
}

static bool in_joint_range(const RobotArmConfig& config, const JointAngles& q) {
    for(int i = 0; i < 6; i++) {
        if(q.angles[i] < config.joint_limits[i][0] - 0.01f || 
           q.angles[i] > config.joint_limits[i][1] + 0.01f)
            return false;
    }
    return true;
}

static float cost_solution(const JointAngles& q, 
                         const JointAngles& qnom, 
                         const std::array<float, 6>& weight) {
    float cost = 0.0;
    for(int i = 0; i < 6; i++) {
        const float diff = q.angles[i] - qnom.angles[i];
        cost += weight[i] * diff * diff;
    }
    return cost;
}

bool find_optimal_solution(const RobotArmConfig& config,
                         const IKSolutions& solutions,
                         const JointAngles& nominal,
                         const std::array<float, 6>& weights,
                         JointAngles& optimal) 
{
    float min_cost = std::numeric_limits<float>::infinity();
    bool found = false;

    for(int i = 0; i < solutions.count; i++) {
        const JointAngles& candidate = solutions.solutions[i];

        if(!in_joint_range(config, candidate)) continue;

        const float current_cost = cost_solution(candidate, nominal, weights);

        if(current_cost < min_cost) {
            min_cost = current_cost;
            optimal = candidate;
            found = true;
        }
    }
    return found;
}

void init_robot_arm_config(RobotArmConfig& config, const char* arm_mode) {
    float s1 = 875, s2 = 162.5, s3 = 210, s4 = 260, s5 = 410;
    if(strcmp(arm_mode, "L") == 0) s2 = -162.5;

    config.dh[0] = DHParameters(0,      0,  s1,         0);
    config.dh[1] = DHParameters(M_PI_2, 0,  s2, M_PI_2);
    config.dh[2] = DHParameters(M_PI_2, 0,   0, M_PI);
    config.dh[3] = DHParameters(0,     s3,   0, -M_PI_2);
    config.dh[4] = DHParameters(-M_PI_2,0,  s4, M_PI_2);
    config.dh[5] = DHParameters(M_PI_2, 0,   0, 0);
    config.dh[6] = DHParameters(-M_PI_2,0,   0, -M_PI_2);

    if(strcmp(arm_mode, "L") == 0) {
        config.joint_limits[0] = {-M_PI,         M_PI};
        config.joint_limits[1] = {-40*M_PI/180,  220*M_PI/180};
        config.joint_limits[2] = {-130*M_PI/180, 130*M_PI/180};
        config.joint_limits[3] = {-M_PI,         M_PI};
        config.joint_limits[4] = {-150*M_PI/180, 150*M_PI/180};
        config.joint_limits[5] = {-M_PI,         M_PI};
    } else {
        config.joint_limits[0] = {-M_PI,         M_PI};
        config.joint_limits[1] = {-220*M_PI/180, 40*M_PI/180};
        config.joint_limits[2] = {-130*M_PI/180, 130*M_PI/180};
        config.joint_limits[3] = {-M_PI,         M_PI};
        config.joint_limits[4] = {-150*M_PI/180, 150*M_PI/180};
        config.joint_limits[5] = {-M_PI,         M_PI};
    }

    config.geometric_params = {s1, s2, s3, s4, s5};
}

// Static member for accumulator matrix
static std::array<std::array<float, 4>, 4> T_accum = {0};

void forward_kinematic(const RobotArmConfig& config, 
                      const JointAngles& q, 
                      Pose& result) {
    std::array<std::array<std::array<float, 4>, 4>, 7> T;

    for(int i = 0; i < 7; i++) {
        float alpha = config.dh[i].alpha;
        float a = config.dh[i].a;
        float d = config.dh[i].d;
        float theta = (i == 0) ? config.dh[i].theta_offset 
                              : q.angles[i - 1] + config.dh[i].theta_offset;
        
        float ct = cos(theta), st = sin(theta);
        float ca = cos(alpha), sa = sin(alpha);

        T[i] = {{{ct,     -st,    0,      a},
                 {st*ca,   ct*ca, -sa,    -d*sa},
                 {st*sa,   ct*sa,  ca,     d*ca},
                 {0,       0,      0,      1}}};
    }

    T_accum = T[0];
    for(int i = 1; i < 7; i++) {
        std::array<std::array<float, 4>, 4> temp;
        matrix_multiply(T_accum, T[i], temp);
        T_accum = temp;
    }

    std::array<float, 4> L = {0, 0, config.geometric_params[4], 1};
    for(int i = 0; i < 3; i++) {
        result.position[i] = 0;
        for(int j = 0; j < 4; j++)
            result.position[i] += T_accum[i][j] * L[j];
    }

    std::array<std::array<float, 3>, 3> R = {{
        {T_accum[0][0], T_accum[0][1], T_accum[0][2]},
        {T_accum[1][0], T_accum[1][1], T_accum[1][2]},
        {T_accum[2][0], T_accum[2][1], T_accum[2][2]}
    }};
    
    if (R_to_Euler(R, "ZYX", result.orientation) != 0) {
        result.orientation = {0.0f, 0.0f, 0.0f};
    }
}

bool inverse_kinematic(const RobotArmConfig& config, 
                      const Pose& target,
                      const JointAngles& nominal, 
                      IKSolutions& solutions) {
    const float s1 = config.geometric_params[0];
    const float s2 = config.geometric_params[1];
    const float s3 = config.geometric_params[2];
    const float s4 = config.geometric_params[3];
    const float s5 = config.geometric_params[4];

    float L_effector = s5;
    float px = target.position[0];
    float py = target.position[1];
    float pz = target.position[2];
                     
    std::array<std::array<float, 3>, 3> R;
    Euler_to_R(target.orientation, "ZYX", R);
                     
    float nx = R[0][0], ny = R[1][0], nz = R[2][0];
    float ox = R[0][1], oy = R[1][1], oz = R[2][1];
    float ax = R[0][2], ay = R[1][2], az = R[2][2];

    float x = px - ax * L_effector;
    float y = py - ay * L_effector;
    float z = pz - az * L_effector;        

    // get joint1
    std::array<float, 2> theta1;
    float A = sqrt(x*x + (s1 - z)*(s1 - z));
    float B = y + s2;

    if (fabs(z - s1) < 1e-9 && fabs(x) < 1e-9) {
        theta1[0] = theta1[1] = nominal.angles[0];
    } else {
        theta1[0] = atan2(x/A, (s1 - z)/A);
        theta1[1] = atan2(-x/A, -(s1 - z)/A);
    }        

    // get joint3
    float Cosq3 = (A*A + B*B - s3*s3 - s4*s4) / (2*s3*s4);
    Cosq3 = std::max(-1.0f, std::min(1.0f, Cosq3));
    float Sinq3 = sqrt(1 - Cosq3*Cosq3);
    std::array<float, 2> theta3 = {atan2(Sinq3, Cosq3), atan2(-Sinq3, Cosq3)};

    // get joint2
    std::array<float, 4> theta2;
    for (int i = 0; i < 2; i++) {
        float C = s4*cos(theta3[i]) + s3;
        float D = s4*sin(theta3[i]);
        theta2[2*i]   = atan2(B*C - A*D, A*C + B*D);
        theta2[2*i+1] = atan2(B*C + A*D, -A*C + B*D);
    }

    std::array<std::array<float, 3>, 8> solution123 = {{
        {theta1[0], theta2[0], theta3[0]}, {theta1[0], theta2[2], theta3[1]},
        {theta1[0], theta2[1], theta3[0]}, {theta1[0], theta2[3], theta3[1]},
        {theta1[1], theta2[0], theta3[0]}, {theta1[1], theta2[2], theta3[1]},
        {theta1[1], theta2[1], theta3[0]}, {theta1[1], theta2[3], theta3[1]}
    }};        

    // get joint456        
    int cnt = 0;
    for (int i = 0; i < 8; i++) {
        float th1 = solution123[i][0];
        float th2 = solution123[i][1];
        float th3 = solution123[i][2];
        
        float sth23 = sin(th2+th3);
        float cth23 = cos(th2+th3);

        float r23 = ay*sth23 - az*cth23*cos(th1) + ax*cth23*sin(th1);
        float r12 = ox*sth23*sin(th1) - oz*sth23*cos(th1) - oy*cth23;
        float r32 = ox*cos(th1) + oz*sin(th1);
        float r13 = ax*sth23*sin(th1) - az*sth23*cos(th1) - ay*cth23;
        float r33 = ax*cos(th1) + az*sin(th1);
        float r21 = ny*sth23 - nz*cth23*cos(th1) + nx*cth23*sin(th1);
        float r22 = oy*sth23 - oz*cth23*cos(th1) + ox*cth23*sin(th1);

        std::array<float, 2> theta5 = {
            atan2(sqrt(1 - r23*r23), r23),
            atan2(-sqrt(1 - r23*r23), r23)
        };

        for (int t5 = 0; t5 < 2; t5++) {
            float th4, th6;
            float th5 = theta5[t5];
            
            if(fabs(sin(th5)) < 1e-6) {
                th4 = nominal.angles[3];
                th6 = atan2(-r12, -r32) - th4;
            } else if(fabs(fabs(th5) - M_PI) < 1e-6) {
                th4 = nominal.angles[3];
                th6 = th4 - atan2(r12, r32);
            } else {
                th4 = atan2(r13/sin(th5), r33/sin(th5));
                th6 = atan2(r21/sin(th5), r22/sin(th5));
            }

            std::array<std::array<float, 2>, 6> shifts = {{
                {0,0}, {0,2*M_PI}, {0,-2*M_PI},
                {0,-2*M_PI}, {-2*M_PI,0}, {-2*M_PI,2*M_PI}
            }};

            for(int s = 0; s < 6; s++) {
                if(cnt < 96) {
                    solutions.solutions[cnt].angles[0] = th1;
                    solutions.solutions[cnt].angles[1] = th2;
                    solutions.solutions[cnt].angles[2] = th3;
                    solutions.solutions[cnt].angles[3] = th4;
                    solutions.solutions[cnt].angles[4] = th5 + shifts[s][0];
                    solutions.solutions[cnt].angles[5] = th6 + shifts[s][1];
                    cnt++;
                }
            }
        }
    }
    solutions.count = cnt;
    return (cnt > 0);
}

} // namespace RobotArm 