/*
 * @file ultrasonic.h
 * @brief 超声波测距模块头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

/* 超声波传感器数量 */
#define US_SENSOR_COUNT    3

/* 超声波传感器位置 */
typedef enum {
    US_FRONT = 0,   // 前方
    US_LEFT  = 1,   // 左方
    US_RIGHT = 2    // 右方
} US_Position_t;

/* ========== 函数声明 ========== */

void Ultrasonic_Init(void);
float Ultrasonic_Read(US_Position_t pos);
void Ultrasonic_ReadAll(float *distances);
float Ultrasonic_GetDistance(US_Position_t pos);

#endif /* __ULTRASONIC_H */
