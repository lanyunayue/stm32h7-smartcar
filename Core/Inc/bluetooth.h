/*
 * @file bluetooth.h
 * @brief HC-05 蓝牙模块驱动头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

#define BT_RX_BUF_SIZE     64
#define BT_CMD_BUF_SIZE    32

/* 蓝牙命令类型 */
typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_FORWARD,      // 前进
    BT_CMD_BACKWARD,     // 后退
    BT_CMD_LEFT,         // 左转
    BT_CMD_RIGHT,        // 右转
    BT_CMD_STOP,         // 停止
    BT_CMD_SPEED_UP,     // 加速
    BT_CMD_SPEED_DOWN,   // 减速
    BT_CMD_MODE_TRACE,   // 切换循迹模式
    BT_CMD_MODE_AVOID,   // 切换避障模式
} BT_Cmd_t;

/* ========== 函数声明 ========== */

void Bluetooth_Init(UART_HandleTypeDef *huart);
void Bluetooth_SendString(const char *str);
void Bluetooth_SendStatus(void);
BT_Cmd_t Bluetooth_GetCommand(void);
void Bluetooth_IRQHandler(void);

#endif /* __BLUETOOTH_H */
