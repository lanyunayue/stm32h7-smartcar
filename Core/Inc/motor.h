/*
 * @file motor.h
 * @brief 电机驱动头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 电机方向 */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} MotorDir_t;

/* 电机ID */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorID_t;

/* 电机配置结构体 */
typedef struct {
    TIM_HandleTypeDef *htim_pwm;  // PWM 定时器
    uint32_t pwm_channel;         // PWM 通道
    GPIO_TypeDef *dir_port1;      // 方向控制引脚1端口
    uint16_t dir_pin1;            // 方向控制引脚1
    GPIO_TypeDef *dir_port2;      // 方向控制引脚2端口
    uint16_t dir_pin2;            // 方向控制引脚2
} MotorConfig_t;

/* ========== 函数声明 ========== */

void Motor_Init(void);
void Motor_SetSpeed(MotorID_t motor, float speed_percent);
void Motor_SetDirection(MotorID_t motor, MotorDir_t dir);
void Motor_Stop(MotorID_t motor);
void Motor_StopAll(void);
void Motor_Drive(float left_speed, float right_speed);

#endif /* __MOTOR_H */
