#include "TrajectoryInterpolator.h"
#include <cstring>
#include <algorithm>
#include <cmath>

TrajectoryInterpolator::TrajectoryInterpolator() :
    queue_head_(0), queue_tail_(0), queue_size_(0)
{
    // 清空轨迹队列
    for (uint8_t i = 0; i < MAX_QUEUE_SIZE; i++) {
        trajectory_queue_[i] = TrajectoryPoint();
    }
}

void TrajectoryInterpolator::init(float sample_time, float max_velocity, float max_acceleration)
{
    // 设置插值参数
    params_.Ts = sample_time;
    params_.Vmax = max_velocity;
    params_.Amax = max_acceleration;
    
    // 计算滤波器参数
    calculate_filter_coefficients();
    
    // 重置状态
    state_ = InterpolationState();
    state_.m = params_.m;
    
    // 清空队列
    clear_queue();
}

void TrajectoryInterpolator::set_initial_position(const float initial_pos[JOINT_DOF])
{
    // 设置初始位置
    copy_array(state_.cmd0, initial_pos);
    copy_array(state_.q0, initial_pos);
    
    // 用初始位置填满滑动窗口
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        for (uint8_t j = 0; j < state_.m; j++) {
            state_.win[i][j] = initial_pos[i];
        }
    }
}

bool TrajectoryInterpolator::add_trajectory_point(const float target_pos[JOINT_DOF], bool need_stop)
{
    if (queue_size_ >= MAX_QUEUE_SIZE) {
        return false; // 队列已满
    }
    
    // 创建新的轨迹点
    TrajectoryPoint& point = trajectory_queue_[queue_tail_];
    copy_array(point.position, target_pos);
    point.is_valid = true;
    point.need_stop = need_stop;
    
    // 更新队列索引
    queue_tail_ = (queue_tail_ + 1) % MAX_QUEUE_SIZE;
    queue_size_++;
    
    return true;
}

bool TrajectoryInterpolator::add_trajectory_points(const float points[][JOINT_DOF], uint8_t count, bool last_point_stop)
{
    if (queue_size_ + count > MAX_QUEUE_SIZE) {
        return false; // 队列空间不足
    }
    
    // 批量添加轨迹点
    for (uint8_t i = 0; i < count; i++) {
        bool need_stop = (i == count - 1) ? last_point_stop : false;
        if (!add_trajectory_point(points[i], need_stop)) {
            return false;
        }
    }
    
    return true;
}

void TrajectoryInterpolator::clear_queue()
{
    queue_head_ = 0;
    queue_tail_ = 0;
    queue_size_ = 0;
    
    for (uint8_t i = 0; i < MAX_QUEUE_SIZE; i++) {
        trajectory_queue_[i].is_valid = false;
    }
}

bool TrajectoryInterpolator::update(float output_pos[JOINT_DOF])
{
    // 检查是否需要启动新段(获取队列中的下一个目标位置)
    if (state_.is_terminated && queue_size_ > 0) {
        TrajectoryPoint next_point;
        if (get_next_trajectory_point(next_point)) {
            seg_init(next_point.position, next_point.need_stop);
        }
    }
    
    // 如果没有活动段，保持当前位置
    if (state_.is_terminated) {
        copy_array(output_pos, state_.cmd0);
        return true; // 段已结束
    }
    
    // 执行段更新，返回插值结果
    bool segment_finished = seg_update(output_pos);
    return segment_finished;
}

bool TrajectoryInterpolator::has_pending_trajectory() const
{
    return queue_size_ > 0 || !state_.is_terminated;
}

bool TrajectoryInterpolator::is_segment_finished() const
{
    return state_.is_terminated;
}

uint8_t TrajectoryInterpolator::get_queue_size() const
{
    return queue_size_;
}

void TrajectoryInterpolator::get_interpolation_status(bool& is_active, uint32_t& current_step, uint32_t& total_steps) const
{
    is_active = !state_.is_terminated;
    current_step = state_.n;
    total_steps = state_.N;
}

void TrajectoryInterpolator::calculate_filter_coefficients()
{
    // 计算滤波器参数：k = ceil(Vmax/Amax/Ts)
    params_.k = static_cast<uint8_t>(std::ceil(params_.Vmax / params_.Amax / params_.Ts));
    if (params_.k < 1) params_.k = 1;
    if (params_.k > MAX_FILTER_SIZE/2) params_.k = MAX_FILTER_SIZE/2;
    
    params_.m = 2 * params_.k - 1;
    if (params_.m > MAX_FILTER_SIZE) params_.m = MAX_FILTER_SIZE;
    
    // 生成三角窗滤波器：conv(ones(1,k)/k, ones(1,k)/k)
    float temp_filter[MAX_FILTER_SIZE] = {0};
    
    // 第一个矩形窗：ones(1,k)/k
    for (uint8_t i = 0; i < params_.k; i++) {
        temp_filter[i] = 1.0f / params_.k;
    }
    
    // 与第二个矩形窗卷积
    for (uint8_t i = 0; i < params_.m; i++) {
        params_.MA_filter[i] = 0.0f;
        for (uint8_t j = 0; j < params_.k; j++) {
            if (i >= j && (i - j) < params_.k) {
                params_.MA_filter[i] += temp_filter[j] * (1.0f / params_.k);
            }
        }
    }
    
    // 归一化（理论上已经归一化，但确保数值稳定）
    float sum = 0.0f;
    for (uint8_t i = 0; i < params_.m; i++) {
        sum += params_.MA_filter[i];
    }
    if (sum > 0.001f) {
        for (uint8_t i = 0; i < params_.m; i++) {
            params_.MA_filter[i] /= sum;
        }
    }
}

void TrajectoryInterpolator::seg_init(const float next_q[JOINT_DOF], bool need_tail)
{
    // 计算关键点差值
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        state_.dq[i] = next_q[i] - state_.cmd0[i];
    }
    
    // 计算线性区点数
    float max_dq = max_abs_array(state_.dq);
    state_.Nlin = static_cast<uint32_t>(std::max(1.0f, std::ceil(max_dq / params_.Vmax / params_.Ts)));
    
    // 首段比后续段多1个点（包含末端）
    if (state_.first_command) {
        state_.Nlin_eff = state_.Nlin + 1;
    } else {
        state_.Nlin_eff = state_.Nlin;
    }
    
    // 末段补充尾部恒值点
    uint32_t tail = need_tail ? state_.m : 0;
    state_.N = state_.Nlin_eff + tail;
    
    // 首段：用起始指令预填窗口
    if (state_.first_command) {
        for (uint8_t i = 0; i < JOINT_DOF; i++) {
            for (uint8_t j = 0; j < state_.m; j++) {
                state_.win[i][j] = state_.cmd0[i];
            }
        }
        state_.first_command = false;
    }
    
    state_.n = 0;
    state_.is_terminated = false;
}

bool TrajectoryInterpolator::seg_update(float q_out[JOINT_DOF])
{
    // 1) 生成一帧原始线性样本
    float ratio;
    if (state_.n < state_.Nlin_eff) {
        ratio = static_cast<float>(state_.n) / static_cast<float>(state_.Nlin);
    } else {
        ratio = 1.0f; // 尾部恒值
    }
    
    float x[JOINT_DOF];
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        x[i] = state_.cmd0[i] + ratio * state_.dq[i];
    }
    
    // 2) 滑动窗口FIFO
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        // 左移窗口
        for (uint8_t j = 0; j < state_.m - 1; j++) {
            state_.win[i][j] = state_.win[i][j + 1];
        }
        // 添加新样本
        state_.win[i][state_.m - 1] = x[i];
    }
    
    // 3) 三角窗平滑
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        q_out[i] = 0.0f;
        for (uint8_t j = 0; j < state_.m; j++) {
            q_out[i] += state_.win[i][j] * params_.MA_filter[j];
        }
    }
    
    // 4) 步进 & 判定本段结束
    state_.n++;
    if (state_.n >= state_.N) {
        state_.is_terminated = true;
        // 更新"上一关键点"
        for (uint8_t i = 0; i < JOINT_DOF; i++) {
            state_.cmd0[i] += state_.dq[i];
        }
        copy_array(state_.q0, q_out);
        return true; // 段结束
    }
    
    return false; // 段继续
}

bool TrajectoryInterpolator::get_next_trajectory_point(TrajectoryPoint& point)
{
    if (queue_size_ == 0) {
        return false;
    }
    
    point = trajectory_queue_[queue_head_];
    queue_head_ = (queue_head_ + 1) % MAX_QUEUE_SIZE;
    queue_size_--;
    
    return point.is_valid;
}

float TrajectoryInterpolator::max_abs_array(const float arr[JOINT_DOF]) const
{
    float max_val = 0.0f;
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        float abs_val = std::abs(arr[i]);
        if (abs_val > max_val) {
            max_val = abs_val;
        }
    }
    return max_val;
}

void TrajectoryInterpolator::copy_array(float dest[JOINT_DOF], const float src[JOINT_DOF]) const
{
    for (uint8_t i = 0; i < JOINT_DOF; i++) {
        dest[i] = src[i];
    }
} 