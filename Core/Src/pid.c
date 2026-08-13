/*
 * @file pid.c
 * @brief PID 控制器实现
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#include "pid.h"

/**
 * @brief  PID 控制器初始化
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float max_output, float max_integral)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->max_integral = max_integral;

    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;

    pid->integral_separation_enabled = false;
    pid->integral_threshold = 0.0f;
}

/**
 * @brief  设置 PID 目标值
 */
void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  位置式 PID 计算
 */
float PID_Calc_Position(PID_t *pid, float current)
{
    pid->current = current;
    pid->error = pid->target - pid->current;

    /* 比例项 */
    float p_term = pid->kp * pid->error;

    /* 积分项（带积分分离） */
    if (pid->integral_separation_enabled) {
        if (fabsf(pid->error) < pid->integral_threshold) {
            pid->integral += pid->error;
        }
    } else {
        pid->integral += pid->error;
    }

    /* 积分限幅 */
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    float i_term = pid->ki * pid->integral;

    /* 微分项 */
    pid->derivative = pid->error - pid->last_error;
    float d_term = pid->kd * pid->derivative;

    /* 输出计算 */
    pid->output = p_term + i_term + d_term;

    /* 输出限幅 */
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    /* 保存误差 */
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief  增量式 PID 计算
 */
float PID_Calc_Incremental(PID_t *pid, float current)
{
    pid->current = current;
    pid->error = pid->target - pid->current;

    /* 增量式公式: delta = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2)) */
    float delta_p = pid->kp * (pid->error - pid->last_error);
    float delta_i = pid->ki * pid->error;
    float delta_d = pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);

    float delta_output = delta_p + delta_i + delta_d;

    /* 更新历史误差 */
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return delta_output;
}

/**
 * @brief  重置 PID 控制器
 */
void PID_Reset(PID_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  启用/禁用积分分离
 */
void PID_SetIntegralSeparation(PID_t *pid, bool enable, float threshold)
{
    pid->integral_separation_enabled = enable;
    pid->integral_threshold = threshold;
}
