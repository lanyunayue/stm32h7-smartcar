/*
 * @file pid.h
 * @brief PID 控制器头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __PID_H
#define __PID_H

#include "main.h"

/* PID 控制器结构体 */
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数

    float target;       // 目标值
    float current;      // 当前值
    float error;        // 当前误差
    float last_error;   // 上一次误差
    float prev_error;   // 上上次误差（增量式用）

    float integral;     // 积分项累积
    float derivative;   // 微分项

    float output;       // 输出值
    float max_output;   // 输出限幅
    float max_integral; // 积分限幅

    /* 积分分离阈值 */
    float integral_threshold;
    /* 是否使用积分分离 */
    bool integral_separation_enabled;
} PID_t;

/* ========== 函数声明 ========== */

/**
 * @brief  PID 控制器初始化
 * @param  pid: PID 结构体指针
 * @param  kp, ki, kd: PID 参数
 * @param  max_output: 输出最大值
 * @param  max_integral: 积分最大值
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float max_output, float max_integral);

/**
 * @brief  设置 PID 目标值
 */
void PID_SetTarget(PID_t *pid, float target);

/**
 * @brief  位置式 PID 计算
 * @param  pid: PID 结构体指针
 * @param  current: 当前反馈值
 * @return 计算输出
 */
float PID_Calc_Position(PID_t *pid, float current);

/**
 * @brief  增量式 PID 计算
 * @param  pid: PID 结构体指针
 * @param  current: 当前反馈值
 * @return 增量输出
 */
float PID_Calc_Incremental(PID_t *pid, float current);

/**
 * @brief  重置 PID 控制器
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  启用/禁用积分分离
 */
void PID_SetIntegralSeparation(PID_t *pid, bool enable, float threshold);

#endif /* __PID_H */
