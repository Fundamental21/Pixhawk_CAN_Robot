#pragma once

#include <stdint.h>

// 队列大小定义
#define SPARSE_QUEUE_SIZE 100        // 稀疏队列大小
#define DENSE_QUEUE_SIZE 10000      // 稠密队列大小 - 1万帧
#define JOINT_COUNT 6               // 关节数量

// 关节点结构
struct JointPoint {
    float angles[JOINT_COUNT];      // 6个关节角度
};

// 稀疏队列（存储关键点）
struct SparseQueue {
    JointPoint data[SPARSE_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
};

// 稠密队列（存储插值后的轨迹点）
struct DenseQueue {
    JointPoint data[DENSE_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
};

// 轨迹插值类
class CAN_Robot_Interpolation {
public:
    static CAN_Robot_Interpolation* get_singleton();
    
    // 队列操作
    bool sparse_queue_push(const JointPoint& point);
    bool sparse_queue_pop(JointPoint& point);
    bool sparse_queue_is_empty() const;
    bool sparse_queue_is_full() const;
    uint8_t sparse_queue_count() const;
    
    bool dense_queue_push(const JointPoint& point);
    bool dense_queue_pop(JointPoint& point);
    bool dense_queue_is_empty() const;
    bool dense_queue_is_full() const;
    uint16_t dense_queue_count() const;
    
    // 轨迹生成函数
    bool generate_joint_trajectory(float Ts, float F, float ub_a, uint8_t max_points = 0);
    
    // 初始化
    void init();

private:
    CAN_Robot_Interpolation();
    static CAN_Robot_Interpolation* _singleton;
    
    SparseQueue _sparse_queue;
    DenseQueue _dense_queue;
    
    bool _initialized;
    
    // 内部辅助函数
    void clear_sparse_queue();
    void clear_dense_queue();
    
    // MATLAB函数的C++实现
    bool generateJointTrajectoryAccCont1(float Ts, float F, float ub_a, 
                                       const JointPoint* J_key, uint8_t num_points,
                                       JointPoint* J_smooth, uint16_t& num_smooth_points);
    
    // 辅助函数
    void linspace(float start, float end, uint16_t num, float* result);
    void moving_average_filter(const float* input, uint16_t input_size, 
                              uint16_t window_size, float* output);
    float conv_single(const float* signal, uint16_t sig_len, 
                     const float* kernel, uint16_t ker_len, uint16_t index);
}; 