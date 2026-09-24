/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.3
 * @date    2026-09-24 1.3 增加 UART7 VOFA+ 四轮速度/位置反馈
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "Chassis.h"
#include "usb_device.h"
#include "user_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace
{
constexpr uint8_t VOFA_IMU_CHANNEL_COUNT = 3U;
constexpr uint8_t VOFA_TELEMETRY_CHANNEL_COUNT =
    VOFA_IMU_CHANNEL_COUNT + 2U * CHASSIS_WHEEL_COUNT;


float motor_speed_rpm[CHASSIS_WHEEL_COUNT] = {};
float motor_position_deg[CHASSIS_WHEEL_COUNT] = {};
}

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    EricTool_UART.Set_Data(VOFA_TELEMETRY_CHANNEL_COUNT,
                           (int) &Debug_IMU_Data.Euler_Yaw_rad,
                           (int) &Debug_IMU_Data.Euler_Pitch_rad,
                           (int) &Debug_IMU_Data.Euler_Roll_rad,
                           (int) &motor_speed_rpm[0],
                           (int) &motor_speed_rpm[1],
                           (int) &motor_speed_rpm[2],
                           (int) &motor_speed_rpm[3],
                           (int) &motor_position_deg[0],
                           (int) &motor_position_deg[1],
                           (int) &motor_position_deg[2],
                           (int) &motor_position_deg[3]);
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

        EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(50);
    }
}
