/*
 * @file kalman.h
 * @brief 卡尔曼滤波器头文件（用于姿态解算）
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __KALMAN_H
#define __KALMAN_H

#include "main.h"

/* 一维卡尔曼滤波器结构体 */
typedef struct {
    float x;      // 状态估计值
    float p;      // 估计协方差
    float q;      // 过程噪声协方差
    float r;      // 测量噪声协方差
    float k;      // 卡尔曼增益
} Kalman1D_t;

/* 二阶卡尔曼（用于角度融合）结构体 */
typedef struct {
    float angle;      // 计算出的角度
    float gyro_bias;  // 陀螺仪偏差
    float p[2][2];    // 误差协方差矩阵
    float q_angle;    // 角度过程噪声
    float q_bias;     // 陀螺仪偏差过程噪声
    float r_measure;  // 测量噪声
} KalmanAngle_t;

/* ========== 函数声明 ========== */

/**
 * @brief  一维卡尔曼滤波初始化
 */
void Kalman1D_Init(Kalman1D_t *kf, float q, float r, float initial_value);

/**
 * @brief  一维卡尔曼滤波更新
 */
float Kalman1D_Update(Kalman1D_t *kf, float measurement);

/**
 * @brief  角度卡尔曼滤波初始化（用于加速度计+陀螺仪融合）
 */
void KalmanAngle_Init(KalmanAngle_t *kf, float q_angle, float q_bias, float r_measure);

/**
 * @brief  角度卡尔曼滤波更新
 * @param  kf: 卡尔曼结构体
 * @param  acc_angle: 加速度计计算的角度 (deg)
 * @param  gyro_rate: 陀螺仪角速度 (deg/s)
 * @param  dt: 采样时间 (s)
 * @return 滤波后的角度 (deg)
 */
float KalmanAngle_Update(KalmanAngle_t *kf, float acc_angle, float gyro_rate, float dt);

#endif /* __KALMAN_H */
