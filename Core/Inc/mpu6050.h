/*
 * @file mpu6050.h
 * @brief MPU6050 六轴传感器驱动头文件
 * @author 巫伟鑫
 * @date 2024-08-12
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

/* MPU6050 I2C 地址（AD0引脚接地时为 0x68） */
#define MPU6050_ADDR         0x68U

/* 寄存器地址定义 */
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75

/* 陀螺仪量程 */
typedef enum {
    GYRO_RANGE_250  = 0,  // ±250°/s
    GYRO_RANGE_500  = 1,  // ±500°/s
    GYRO_RANGE_1000 = 2,  // ±1000°/s
    GYRO_RANGE_2000 = 3,  // ±2000°/s
} GyroRange_t;

/* 加速度计量程 */
typedef enum {
    ACCEL_RANGE_2G  = 0,  // ±2g
    ACCEL_RANGE_4G  = 1,  // ±4g
    ACCEL_RANGE_8G  = 2,  // ±8g
    ACCEL_RANGE_16G = 3,  // ±16g
} AccelRange_t;

/* MPU6050 数据结构体 */
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;

    float accel_x;    // g
    float accel_y;    // g
    float accel_z;    // g
    float gyro_x;     // deg/s
    float gyro_y;     // deg/s
    float gyro_z;     // deg/s
    float temp;       // °C

    float pitch;      // 俯仰角 (由加速度计计算)
    float roll;       // 横滚角
} MPU6050_t;

/* ========== 函数声明 ========== */

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, GyroRange_t gyro_range, AccelRange_t accel_range);
uint8_t MPU6050_ReadID(I2C_HandleTypeDef *hi2c);
void MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_t *data);
void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *data);
void MPU6050_CalcAngle(MPU6050_t *data);

#endif /* __MPU6050_H */
