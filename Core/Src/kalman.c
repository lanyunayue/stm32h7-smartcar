/*
 * @file kalman.c
 * @brief 卡尔曼滤波器实现
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#include "kalman.h"

/**
 * @brief  一维卡尔曼滤波初始化
 */
void Kalman1D_Init(Kalman1D_t *kf, float q, float r, float initial_value)
{
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0f;
    kf->k = 0.0f;
}

/**
 * @brief  一维卡尔曼滤波更新
 */
float Kalman1D_Update(Kalman1D_t *kf, float measurement)
{
    /* 1. 预测 */
    kf->p += kf->q;  // 预测协方差

    /* 2. 更新 */
    kf->k = kf->p / (kf->p + kf->r);                 // 卡尔曼增益
    kf->x += kf->k * (measurement - kf->x);          // 状态更新
    kf->p = (1.0f - kf->k) * kf->p;                  // 协方差更新

    return kf->x;
}

/**
 * @brief  角度卡尔曼滤波初始化
 */
void KalmanAngle_Init(KalmanAngle_t *kf, float q_angle, float q_bias, float r_measure)
{
    kf->angle = 0.0f;
    kf->gyro_bias = 0.0f;

    /* 初始化协方差矩阵 */
    kf->p[0][0] = 1.0f;  kf->p[0][1] = 0.0f;
    kf->p[1][0] = 0.0f;  kf->p[1][1] = 1.0f;

    kf->q_angle = q_angle;
    kf->q_bias = q_bias;
    kf->r_measure = r_measure;
}

/**
 * @brief  角度卡尔曼滤波更新（加速度计 + 陀螺仪融合）
 */
float KalmanAngle_Update(KalmanAngle_t *kf, float acc_angle, float gyro_rate, float dt)
{
    /* ===== 预测阶段 ===== */

    /* 先验角度估计: 角度 = 上一时刻角度 + (角速度 - 偏差) * dt */
    kf->angle += (gyro_rate - kf->gyro_bias) * dt;

    /* 先验误差协方差矩阵 P(k|k-1) = A*P*A^T + Q */
    kf->p[0][0] += -dt * (kf->p[1][0] + kf->p[0][1]) + kf->q_angle * dt;
    kf->p[0][1] += -dt * kf->p[1][1];
    kf->p[1][0] += -dt * kf->p[1][1];
    kf->p[1][1] += kf->q_bias * dt;

    /* ===== 更新阶段 ===== */

    /* 计算新息（测量残差）: y = z - H*x^ */
    float y = acc_angle - kf->angle;

    /* 计算新息协方差: S = H*P*H^T + R = P[0][0] + R */
    float s = kf->p[0][0] + kf->r_measure;

    /* 计算卡尔曼增益: K = P*H^T / S */
    float k0 = kf->p[0][0] / s;
    float k1 = kf->p[1][0] / s;

    /* 更新状态估计 */
    kf->angle += k0 * y;
    kf->gyro_bias += k1 * y;

    /* 更新协方差矩阵: P = (I - K*H) * P */
    float p00_temp = kf->p[0][0];
    float p01_temp = kf->p[0][1];

    kf->p[0][0] -= k0 * p00_temp;
    kf->p[0][1] -= k0 * p01_temp;
    kf->p[1][0] -= k1 * p00_temp;
    kf->p[1][1] -= k1 * p01_temp;

    return kf->angle;
}
