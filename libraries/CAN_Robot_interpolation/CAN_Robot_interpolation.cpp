#include "CAN_Robot_interpolation.h"
#include <cstring>
#include <cmath>
#include <algorithm>

// Static singleton instance
CAN_Robot_Interpolation* CAN_Robot_Interpolation::_singleton = nullptr;

CAN_Robot_Interpolation::CAN_Robot_Interpolation() : _initialized(false)
{
    memset(&_sparse_queue, 0, sizeof(_sparse_queue));
    memset(&_dense_queue, 0, sizeof(_dense_queue));
}

CAN_Robot_Interpolation* CAN_Robot_Interpolation::get_singleton()
{
    if (_singleton == nullptr) {
        _singleton = new CAN_Robot_Interpolation();
    }
    return _singleton;
}

void CAN_Robot_Interpolation::init()
{
    if (_initialized) return;
    
    clear_sparse_queue();
    clear_dense_queue();
    _initialized = true;
}

// 稀疏队列操作
bool CAN_Robot_Interpolation::sparse_queue_push(const JointPoint& point)
{
    if (_sparse_queue.count >= SPARSE_QUEUE_SIZE) {
        return false;
    }
    
    _sparse_queue.data[_sparse_queue.tail] = point;
    _sparse_queue.tail = (_sparse_queue.tail + 1) % SPARSE_QUEUE_SIZE;
    _sparse_queue.count++;
    return true;
}

bool CAN_Robot_Interpolation::sparse_queue_pop(JointPoint& point)
{
    if (_sparse_queue.count == 0) {
        return false;
    }
    
    point = _sparse_queue.data[_sparse_queue.head];
    _sparse_queue.head = (_sparse_queue.head + 1) % SPARSE_QUEUE_SIZE;
    _sparse_queue.count--;
    return true;
}

bool CAN_Robot_Interpolation::sparse_queue_is_empty() const
{
    return _sparse_queue.count == 0;
}

bool CAN_Robot_Interpolation::sparse_queue_is_full() const
{
    return _sparse_queue.count >= SPARSE_QUEUE_SIZE;
}

uint8_t CAN_Robot_Interpolation::sparse_queue_count() const
{
    return _sparse_queue.count;
}

// 稠密队列操作
bool CAN_Robot_Interpolation::dense_queue_push(const JointPoint& point)
{
    if (_dense_queue.count >= DENSE_QUEUE_SIZE) {
        return false;
    }
    
    _dense_queue.data[_dense_queue.tail] = point;
    _dense_queue.tail = (_dense_queue.tail + 1) % DENSE_QUEUE_SIZE;
    _dense_queue.count++;
    return true;
}

bool CAN_Robot_Interpolation::dense_queue_pop(JointPoint& point)
{
    if (_dense_queue.count == 0) {
        return false;
    }
    
    point = _dense_queue.data[_dense_queue.head];
    _dense_queue.head = (_dense_queue.head + 1) % DENSE_QUEUE_SIZE;
    _dense_queue.count--;
    return true;
}

bool CAN_Robot_Interpolation::dense_queue_is_empty() const
{
    return _dense_queue.count == 0;
}

bool CAN_Robot_Interpolation::dense_queue_is_full() const
{
    return _dense_queue.count >= DENSE_QUEUE_SIZE;
}

uint16_t CAN_Robot_Interpolation::dense_queue_count() const
{
    return _dense_queue.count;
}

// 轨迹生成主函数
bool CAN_Robot_Interpolation::generate_joint_trajectory(float Ts, float F, float ub_a, uint8_t max_points)
{
    if (!_initialized || sparse_queue_is_empty()) {
        return false;
    }
    
    // 确定要处理的点数
    uint8_t total_points = _sparse_queue.count;
    uint8_t points_to_process = (max_points == 0 || max_points > total_points) ? total_points : max_points;
    
    // 收集指定数量的稀疏队列中的点
    JointPoint key_points[SPARSE_QUEUE_SIZE];
    
    // 复制队列内容（不移除）
    uint8_t temp_head = _sparse_queue.head;
    for (uint8_t i = 0; i < points_to_process; i++) {
        key_points[i] = _sparse_queue.data[temp_head];
        temp_head = (temp_head + 1) % SPARSE_QUEUE_SIZE;
    }
    
    // 生成平滑轨迹
    JointPoint* smooth_points = new JointPoint[DENSE_QUEUE_SIZE];
    uint16_t num_smooth_points = 0;
    
    if (generateJointTrajectoryAccCont1(Ts, F, ub_a, key_points, points_to_process, 
                                       smooth_points, num_smooth_points)) {
        // 将平滑轨迹推入稠密队列
        for (uint16_t i = 0; i < num_smooth_points && !dense_queue_is_full(); i++) {
            dense_queue_push(smooth_points[i]);
        }
        
        // 从稀疏队列中移除已处理的点
        for (uint8_t i = 0; i < points_to_process; i++) {
            JointPoint temp;
            sparse_queue_pop(temp);  // 移除已处理的点
        }
        
        delete[] smooth_points;
        return true;
    }
    
    delete[] smooth_points;
    return false;
}

// MATLAB函数的C++实现
bool CAN_Robot_Interpolation::generateJointTrajectoryAccCont1(float Ts, float F, float ub_a, 
                                                            const JointPoint* J_key, uint8_t num_points,
                                                            JointPoint* J_smooth, uint16_t& num_smooth_points)
{
    if (num_points < 2) {
        return false;
    }
    
    // 起点
    JointPoint startPoint = J_key[0];
    
    // 创建相对位置数组
    JointPoint* relative_points = new JointPoint[num_points];
    for (uint8_t i = 0; i < num_points; i++) {
        for (uint8_t j = 0; j < JOINT_COUNT; j++) {
            relative_points[i].angles[j] = J_key[i].angles[j] - startPoint.angles[j];
        }
    }
    
    // 分段直线插值
    JointPoint* J_full = new JointPoint[DENSE_QUEUE_SIZE];
    uint16_t full_count = 0;
    
    for (uint8_t i = 0; i < num_points - 1; i++) {
        // 计算该段最大关节位移
        float max_dJ = 0.0f;
        for (uint8_t j = 0; j < JOINT_COUNT; j++) {
            float dJ = fabsf(relative_points[i+1].angles[j] - relative_points[i].angles[j]);
            if (dJ > max_dJ) {
                max_dJ = dJ;
            }
        }
        
        // 计算时间和采样点数
        float T_segment = max_dJ / F;
        uint16_t N = (uint16_t)ceilf(T_segment / Ts);
        if (N == 0) N = 1;
        
        // 生成插值参数
        float* s = new float[N + 1];
        linspace(0.0f, 1.0f, N + 1, s);
        
        // 防止重复末点
        uint16_t end_idx = (i < num_points - 2) ? N : N + 1;
        
        // 线性插值
        for (uint16_t k = 0; k < end_idx && full_count < DENSE_QUEUE_SIZE; k++) {
            for (uint8_t j = 0; j < JOINT_COUNT; j++) {
                J_full[full_count].angles[j] = (1.0f - s[k]) * relative_points[i].angles[j] + 
                                              s[k] * relative_points[i+1].angles[j];
            }
            full_count++;
        }
        
        delete[] s;
    }
    
    // 双长方窗均值滤波
    uint16_t k = (uint16_t)ceilf(F / ub_a / Ts);
    if (k == 0) k = 1;
    uint16_t m = 2 * k - 1;
    
    // 创建滤波器
    float* rect = new float[k];
    for (uint16_t i = 0; i < k; i++) {
        rect[i] = 1.0f / k;
    }
    
    // 末端补点
    uint16_t padded_size = full_count + m;
    JointPoint* J_full_padded = new JointPoint[padded_size];
    
    // 复制原始数据
    for (uint16_t i = 0; i < full_count; i++) {
        J_full_padded[i] = J_full[i];
    }
    
    // 补充末端点
    for (uint16_t i = full_count; i < padded_size; i++) {
        J_full_padded[i] = J_full[full_count - 1];
    }
    
    // 对每个关节进行滤波
    for (uint8_t joint = 0; joint < JOINT_COUNT; joint++) {
        // 提取单个关节的数据
        float* joint_data = new float[padded_size];
        for (uint16_t i = 0; i < padded_size; i++) {
            joint_data[i] = J_full_padded[i].angles[joint];
        }
        
        // 两次移动平均滤波
        float* temp_filtered = new float[padded_size];
        moving_average_filter(joint_data, padded_size, k, temp_filtered);
        moving_average_filter(temp_filtered, padded_size, k, joint_data);
        
        // 存回结果
        for (uint16_t i = 0; i < full_count; i++) {
            J_smooth[i].angles[joint] = joint_data[i] + startPoint.angles[joint];
        }
        
        delete[] joint_data;
        delete[] temp_filtered;
    }
    
    num_smooth_points = full_count;
    
    // 清理内存
    delete[] relative_points;
    delete[] J_full;
    delete[] J_full_padded;
    delete[] rect;
    
    return true;
}

// 辅助函数实现
void CAN_Robot_Interpolation::linspace(float start, float end, uint16_t num, float* result)
{
    if (num <= 1) {
        if (num == 1) result[0] = start;
        return;
    }
    
    float step = (end - start) / (num - 1);
    for (uint16_t i = 0; i < num; i++) {
        result[i] = start + i * step;
    }
}

void CAN_Robot_Interpolation::moving_average_filter(const float* input, uint16_t input_size, 
                                                   uint16_t window_size, float* output)
{
    for (uint16_t i = 0; i < input_size; i++) {
        float sum = 0.0f;
        uint16_t count = 0;
        
        for (uint16_t j = 0; j < window_size && (i + j) < input_size; j++) {
            sum += input[i + j];
            count++;
        }
        
        output[i] = sum / count;
    }
}

float CAN_Robot_Interpolation::conv_single(const float* signal, uint16_t sig_len, 
                                         const float* kernel, uint16_t ker_len, uint16_t index)
{
    float result = 0.0f;
    for (uint16_t i = 0; i < ker_len; i++) {
        if (index >= i && (index - i) < sig_len) {
            result += signal[index - i] * kernel[i];
        }
    }
    return result;
}

// 内部辅助函数
void CAN_Robot_Interpolation::clear_sparse_queue()
{
    _sparse_queue.head = 0;
    _sparse_queue.tail = 0;
    _sparse_queue.count = 0;
}

void CAN_Robot_Interpolation::clear_dense_queue()
{
    _dense_queue.head = 0;
    _dense_queue.tail = 0;
    _dense_queue.count = 0;
} 