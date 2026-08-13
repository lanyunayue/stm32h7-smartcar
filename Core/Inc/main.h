/*
 * @file main.h
 * @brief STM32H7 智能车主程序头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ========== 系统配置 ========== */
#define SYSTEM_CLOCK_FREQ     480000000U   // 系统时钟 480MHz
#define SYSTICK_FREQ          1000U         // 滴答定时器 1kHz

/* ========== 运行模式定义 ========== */
typedef enum {
    MODE_IDLE = 0,        // 待机模式
    MODE_TRACE,           // 循迹模式
    MODE_OBSTACLE_AVOID,  // 避障模式
    MODE_BLUETOOTH,       // 蓝牙遥控模式
    MODE_CRUISE           // 定速巡航模式
} RunMode_t;

/* ========== 全局状态结构体 ========== */
typedef struct {
    RunMode_t mode;       // 当前运行模式
    float speed_left;     // 左轮速度 (cm/s)
    float speed_right;    // 右轮速度 (cm/s)
    float speed_target;   // 目标速度 (cm/s)
    float battery_volt;   // 电池电压 (V)
    float pitch;          // 俯仰角 (deg)
    float roll;           // 横滚角 (deg)
    float yaw;            // 偏航角 (deg)
    float distance_front; // 前方距离 (cm)
    float distance_left;  // 左方距离 (cm)
    float distance_right; // 右方距离 (cm)
    uint8_t trace_status; // 循迹传感器状态
    bool system_ready;    // 系统就绪标志
} SmartCar_t;

/* 全局状态变量 */
extern SmartCar_t g_car;

/* ========== 函数声明 ========== */
void SystemClock_Config(void);
void Error_Handler(void);
void SmartCar_Init(void);
void SmartCar_Update(void);
void Mode_Switch(RunMode_t new_mode);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
