#pragma once

#include <stdint.h>
#include <cmath>

// 插值配置常量
#define JOINT_DOF 6                    // 关节自由度数
#define MAX_QUEUE_SIZE 100             // 最大队列长度（100行，每行6个点）
#define MAX_FILTER_SIZE 20             // 最大滤波器窗口大小

// 插值参数结构体
struct InterpolationParams {
    float Ts;           // 采样周期 (s)
    float Vmax;         // 最大角速度 (deg/s)
    float Amax;         // 最大角加速度 (deg/s²)
    uint8_t k;          // 单级MA窗口大小
    uint8_t m;          // 三角窗长度
    float MA_filter[MAX_FILTER_SIZE];  // 三角窗滤波器系数
    
    // 默认构造函数
    InterpolationParams() : 
        Ts(0.01f), Vmax(10.0f), Amax(100.0f), k(1), m(1) {
        // 初始化滤波器为单位冲激
        for (uint8_t i = 0; i < MAX_FILTER_SIZE; i++) {
            MA_filter[i] = (i == 0) ? 1.0f : 0.0f;
        }
    }
};

// 插值状态结构体
struct InterpolationState {
    bool first_command;                    // 首段标记
    bool is_terminated;                    // 段结束标记
    float cmd0[JOINT_DOF];                 // 上一关键点位置
    float q0[JOINT_DOF];                   // 当前位置（仅作日志）
    uint8_t m;                             // 滑动窗口长度
    float win[JOINT_DOF][MAX_FILTER_SIZE]; // 滑动窗口 DOF×m
    
    // 当前段参数
    uint32_t Nlin;         // 原始几何意义上的线性点数
    uint32_t Nlin_eff;     // 有效线性采样点数
    uint32_t N;            // 总采样点数（包含尾部）
    float dq[JOINT_DOF];   // 关键点之差
    uint32_t n;            // 当前计数器
    
    // 默认构造函数
    InterpolationState() : 
        first_command(true), is_terminated(true), m(1), 
        Nlin(0), Nlin_eff(0), N(0), n(0) {
        // 初始化数组
        for (uint8_t i = 0; i < JOINT_DOF; i++) {
            cmd0[i] = 0.0f;
            q0[i] = 0.0f;
            dq[i] = 0.0f;
            for (uint8_t j = 0; j < MAX_FILTER_SIZE; j++) {
                win[i][j] = 0.0f;
            }
        }
    }
};

// 轨迹队列点结构体
struct TrajectoryPoint {
    float position[JOINT_DOF];  // 关节位置
    bool is_valid;              // 是否有效
    bool need_stop;             // 是否需要完全停止
    
    TrajectoryPoint() : is_valid(false), need_stop(false) {
        for (uint8_t i = 0; i < JOINT_DOF; i++) {
            position[i] = 0.0f;
        }
    }
}; 