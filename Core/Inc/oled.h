/*
 * @file oled.h
 * @brief 0.96寸 OLED SSD1306 驱动头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_I2C_ADDR     0x78U   // OLED I2C 地址
#define OLED_WIDTH        128     // 宽度
#define OLED_HEIGHT       64      // 高度

/* ========== 函数声明 ========== */

void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode);
void OLED_Refresh(void);

/* 显示界面函数 */
void OLED_ShowMainPage(void);
void OLED_UpdateSpeed(void);
void OLED_UpdateBattery(void);
void OLED_UpdateMode(void);
void OLED_ShowSplash(void);

#endif /* __OLED_H */
