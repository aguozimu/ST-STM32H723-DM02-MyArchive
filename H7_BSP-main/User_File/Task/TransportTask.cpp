/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.4
 * @date    2026-09-24 1.4 UART7 增加 INS 三个位姿的 x/y/yaw
 * @date    2026-09-24 1.3 增加 UART7 VOFA+ 四轮速度/位置反馈
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "Chassis.h"
#include "INS.h"
#include "usb_device.h"
#include "user_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace
{
constexpr uint8_t VOFA_IMU_CHANNEL_COUNT = 3U;
constexpr uint8_t VOFA_INS_CHANNEL_COUNT = 3U * 3U;
constexpr uint8_t VOFA_TELEMETRY_CHANNEL_COUNT =
    VOFA_IMU_CHANNEL_COUNT + 2U * CHASSIS_WHEEL_COUNT + VOFA_INS_CHANNEL_COUNT;


float motor_speed_rpm[CHASSIS_WHEEL_COUNT] = {};
float motor_position_deg[CHASSIS_WHEEL_COUNT] = {};
// VOFA+ 0 基通道 11~19：三个 INS 位姿分别发送 x / y / yaw。
float wheel_position_data[3] = {};
float gyro_position_data[3] = {};
float complementary_position_data[3] = {};
}

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    /* UART7 justfloat 顺序：0~2 IMU 原始加速度，3~6 轮速，7~10 轮位置，11~19 三个 INS 位姿。 */
    EricTool_UART.Set_Data(VOFA_TELEMETRY_CHANNEL_COUNT,
                           (int) &Debug_IMU_Data.Original_Accel_X_m_s2,
                           (int) &Debug_IMU_Data.Original_Accel_Y_m_s2,
                           (int) &Debug_IMU_Data.Original_Accel_Z_m_s2,
                           (int) &motor_speed_rpm[0],
                           (int) &motor_speed_rpm[1],
                           (int) &motor_speed_rpm[2],
                           (int) &motor_speed_rpm[3],
                           (int) &motor_position_deg[0],
                           (int) &motor_position_deg[1],
                           (int) &motor_position_deg[2],
                           (int) &motor_position_deg[3],
                           (int) &wheel_position_data[0],
                           (int) &wheel_position_data[1],
                           (int) &wheel_position_data[2],
                           (int) &gyro_position_data[0],
                           (int) &gyro_position_data[1],
                           (int) &gyro_position_data[2],
                           (int) &complementary_position_data[0],
                           (int) &complementary_position_data[1],
                           (int) &complementary_position_data[2]);
    for (;;)
    {
        APP_Chassis.Process();
        for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            const Struct_ZDTMotor_Feedback feedback =
                APP_Chassis.GetMotorFeedback(i);
            motor_speed_rpm[i] = feedback.speed_rpm;
            motor_position_deg[i] = feedback.position_deg;
        }

        /* 每个发送周期复制一次三个 INS 快照，通道 11~19 分别对应下表顺序。 */
        const Struct_INS_Pose wheel_position = APP_INS.GetWheelPosition();
        wheel_position_data[0] = wheel_position.x_m;
        wheel_position_data[1] = wheel_position.y_m;
        wheel_position_data[2] = wheel_position.yaw_rad;

        const Struct_INS_Pose gyro_position = APP_INS.GetGyroPosition();
        gyro_position_data[0] = gyro_position.x_m;
        gyro_position_data[1] = gyro_position.y_m;
        gyro_position_data[2] = gyro_position.yaw_rad;

        const Struct_INS_Pose complementary_position =
            APP_INS.GetComplementaryPosition();
        complementary_position_data[0] = complementary_position.x_m;
        complementary_position_data[1] = complementary_position.y_m;
        complementary_position_data[2] = complementary_position.yaw_rad;

        EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(50);
    }
}
