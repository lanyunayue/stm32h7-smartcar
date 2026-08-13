/*
 * @file main.c
 * @brief STM32H7 智能车主程序
 * @author 巫伟鑫
 * @date 2024-08-12
 *
 * 项目：多传感器融合智能车控制系统
 * 主控：STM32H743VIT6 (Cortex-M7, 480MHz)
 * 传感器：MPU6050, HC-SR04 ×3, 红外循迹 ×5, 编码器 ×2
 * 执行器：直流减速电机 ×2, 舵机 ×1
 * 通信：HC-05 蓝牙模块
 * 显示：0.96寸 OLED
 */

#include "main.h"
#include "pid.h"
#include "kalman.h"
#include "mpu6050.h"
#include "motor.h"
#include "ultrasonic.h"
#include "oled.h"
#include "bluetooth.h"

/* ========== 全局变量 ========== */
SmartCar_t g_car;

/* PID 控制器 */
static PID_t g_pid_speed_left;
static PID_t g_pid_speed_right;
static PID_t g_pid_steer;

/* 卡尔曼滤波器 */
static KalmanAngle_t g_kalman_pitch;
static KalmanAngle_t g_kalman_roll;

/* I2C/UART 句柄（由 CubeMX 生成，这里声明外部引用） */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/* 系统时基计数 */
static volatile uint32_t g_tick_count = 0;

/* ========== 函数声明 ========== */
static void System_Init(void);
static void Sensor_Task(void);
static void Control_Task(void);
static void Display_Task(void);
static void Bluetooth_Task(void);

/**
  * @brief  主函数
  */
int main(void)
{
    /* HAL 库初始化 */
    HAL_Init();

    /* 系统时钟配置 */
    SystemClock_Config();

    /* 系统外设与模块初始化 */
    System_Init();

    /* 显示开机画面 */
    OLED_ShowSplash();
    HAL_Delay(1500);
    OLED_Clear();

    /* 主循环 */
    while (1) {
        /* 10ms 任务：传感器数据采集 */
        Sensor_Task();

        /* 20ms 任务：控制算法 */
        if ((g_tick_count % 2) == 0) {
            Control_Task();
        }

        /* 100ms 任务：显示更新 */
        if ((g_tick_count % 10) == 0) {
            Display_Task();
        }

        /* 50ms 任务：蓝牙通信 */
        if ((g_tick_count % 5) == 0) {
            Bluetooth_Task();
        }

        HAL_Delay(10);
        g_tick_count++;
    }
}

/**
  * @brief  系统初始化
  */
static void System_Init(void)
{
    /* 初始化全局状态 */
    memset(&g_car, 0, sizeof(SmartCar_t));
    g_car.mode = MODE_IDLE;
    g_car.speed_target = 50.0f;  // 默认目标速度 50cm/s

    /* MPU6050 初始化 */
    MPU6050_Init(&hi2c1, GYRO_RANGE_500, ACCEL_RANGE_2G);

    /* 电机初始化 */
    Motor_Init();

    /* 超声波初始化 */
    Ultrasonic_Init();

    /* OLED 初始化 */
    OLED_Init(&hi2c1);

    /* 蓝牙初始化 */
    Bluetooth_Init(&huart1);

    /* PID 控制器初始化 - 左轮速度环 */
    PID_Init(&g_pid_speed_left,
             0.8f,    // Kp
             0.3f,    // Ki
             0.1f,    // Kd
             100.0f,  // 最大输出
             50.0f);  // 积分限幅

    /* PID 控制器初始化 - 右轮速度环 */
    PID_Init(&g_pid_speed_right,
             0.8f,
             0.3f,
             0.1f,
             100.0f,
             50.0f);

    /* PID 控制器初始化 - 转向控制 */
    PID_Init(&g_pid_steer,
             1.5f,
             0.0f,
             0.5f,
             50.0f,
             20.0f);

    /* 卡尔曼滤波初始化 - 俯仰角 */
    KalmanAngle_Init(&g_kalman_pitch, 0.001f, 0.003f, 0.03f);

    /* 卡尔曼滤波初始化 - 横滚角 */
    KalmanAngle_Init(&g_kalman_roll, 0.001f, 0.003f, 0.03f);

    g_car.system_ready = true;
}

/**
  * @brief  传感器任务
  */
static void Sensor_Task(void)
{
    static MPU6050_t mpu_data;
    float distances[US_SENSOR_COUNT];
    static float last_time = 0.0f;
    float dt = 0.01f;  // 10ms

    /* 读取 MPU6050 */
    MPU6050_ReadAll(&hi2c1, &mpu_data);

    /* 计算加速度计角度 */
    MPU6050_CalcAngle(&mpu_data);

    /* 卡尔曼滤波融合 */
    g_car.pitch = KalmanAngle_Update(&g_kalman_pitch,
                                       mpu_data.pitch,
                                       mpu_data.gyro_y,
                                       dt);
    g_car.roll = KalmanAngle_Update(&g_kalman_roll,
                                      mpu_data.roll,
                                      mpu_data.gyro_x,
                                      dt);

    /* 读取超声波距离 */
    Ultrasonic_ReadAll(distances);
    g_car.distance_front = distances[US_FRONT];
    g_car.distance_left  = distances[US_LEFT];
    g_car.distance_right = distances[US_RIGHT];
}

/**
  * @brief  控制任务
  */
static void Control_Task(void)
{
    float left_output, right_output;
    float speed_diff;

    if (!g_car.system_ready) return;

    switch (g_car.mode) {
        case MODE_TRACE:
            /* 循迹模式 - 基于红外传感器 */
            // speed_diff = Trace_Calculate();
            // PID_SetTarget(&g_pid_steer, speed_diff);
            // 左右轮差速控制
            break;

        case MODE_OBSTACLE_AVOID:
            /* 避障模式 - 基于超声波 */
            if (g_car.distance_front < 20.0f) {
                /* 前方有障碍，转向 */
                if (g_car.distance_left > g_car.distance_right) {
                    Motor_SetDirection(MOTOR_LEFT, MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                } else {
                    Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_RIGHT, MOTOR_BACKWARD);
                }
                Motor_Drive(30.0f, 30.0f);
            } else {
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_Drive(g_car.speed_target, g_car.speed_target);
            }
            break;

        case MODE_BLUETOOTH:
            /* 蓝牙遥控模式 */
            break;

        case MODE_CRUISE:
            /* 定速巡航模式 - PID 调速 */
            PID_SetTarget(&g_pid_speed_left, g_car.speed_target);
            PID_SetTarget(&g_pid_speed_right, g_car.speed_target);

            left_output = PID_Calc_Position(&g_pid_speed_left, g_car.speed_left);
            right_output = PID_Calc_Position(&g_pid_speed_right, g_car.speed_right);

            Motor_Drive(left_output, right_output);
            break;

        default:
            Motor_StopAll();
            break;
    }
}

/**
  * @brief  显示任务
  */
static void Display_Task(void)
{
    OLED_ShowMainPage();
}

/**
  * @brief  蓝牙任务
  */
static void Bluetooth_Task(void)
{
    BT_Cmd_t cmd = Bluetooth_GetCommand();

    switch (cmd) {
        case BT_CMD_FORWARD:
            Mode_Switch(MODE_CRUISE);
            break;
        case BT_CMD_STOP:
            Mode_Switch(MODE_IDLE);
            break;
        case BT_CMD_MODE_TRACE:
            Mode_Switch(MODE_TRACE);
            break;
        case BT_CMD_MODE_AVOID:
            Mode_Switch(MODE_OBSTACLE_AVOID);
            break;
        case BT_CMD_SPEED_UP:
            if (g_car.speed_target < 100.0f)
                g_car.speed_target += 10.0f;
            break;
        case BT_CMD_SPEED_DOWN:
            if (g_car.speed_target > 10.0f)
                g_car.speed_target -= 10.0f;
            break;
        default:
            break;
    }
}

/**
  * @brief  模式切换
  */
void Mode_Switch(RunMode_t new_mode)
{
    if (new_mode == g_car.mode) return;

    /* 退出当前模式 */
    PID_Reset(&g_pid_speed_left);
    PID_Reset(&g_pid_speed_right);
    PID_Reset(&g_pid_steer);
    Motor_StopAll();

    /* 切换到新模式 */
    g_car.mode = new_mode;
}

/**
  * @brief  系统时钟配置
  * @note   此函数通常由 CubeMX 生成
  */
void SystemClock_Config(void)
{
    /* STM32H743 系统时钟配置为 480MHz */
    /* 具体配置由 CubeMX 生成代码完成 */
}

/**
  * @brief  错误处理函数
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* 错误指示：LED 闪烁 */
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        HAL_Delay(200);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 断言失败处理 */
}
#endif
