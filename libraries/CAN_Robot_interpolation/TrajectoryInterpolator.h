#pragma once

#include "InterpolationConfig.h"

class TrajectoryInterpolator {
public:
    // 构造函数
    TrajectoryInterpolator();
    
    // 初始化插值器
    void init(float sample_time, float max_velocity, float max_acceleration);
    
    // 设置初始位置
    void set_initial_position(const float initial_pos[JOINT_DOF]);
    
    // 添加轨迹点到队列
    bool add_trajectory_point(const float target_pos[JOINT_DOF], bool need_stop = false);
    
    // 批量添加轨迹点
    bool add_trajectory_points(const float points[][JOINT_DOF], uint8_t count, bool last_point_stop = false);
    
    // 清空轨迹队列
    void clear_queue();
    
    // 更新插值，返回当前目标位置
    // 返回值：true表示到达目标位置(段结束)，false表示仍在插值过程中
    bool update(float output_pos[JOINT_DOF]);
    
    // 检查是否有待处理的轨迹点
    bool has_pending_trajectory() const;
    
    // 检查当前段是否结束
    bool is_segment_finished() const;
    
    // 获取队列中剩余点数
    uint8_t get_queue_size() const;
    
    // 获取当前插值状态（用于调试）
    void get_interpolation_status(bool& is_active, uint32_t& current_step, uint32_t& total_steps) const;

private:
    // 插值参数和状态
    InterpolationParams params_;
    InterpolationState state_;
    
    // 轨迹队列
    TrajectoryPoint trajectory_queue_[MAX_QUEUE_SIZE];
    uint8_t queue_head_;    // 队列头索引
    uint8_t queue_tail_;    // 队列尾索引
    uint8_t queue_size_;    // 当前队列大小
    
    // 内部方法
    void calculate_filter_coefficients();
    void seg_init(const float next_q[JOINT_DOF], bool need_tail);
    bool seg_update(float q_out[JOINT_DOF]);
    bool get_next_trajectory_point(TrajectoryPoint& point);
    float max_abs_array(const float arr[JOINT_DOF]) const;
    void copy_array(float dest[JOINT_DOF], const float src[JOINT_DOF]) const;
}; 