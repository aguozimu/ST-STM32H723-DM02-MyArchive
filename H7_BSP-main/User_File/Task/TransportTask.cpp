/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.4
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 * @date    2026-09-23 1.3 增加四轮反馈 justfloat 遥测
 * @date    2026-09-23 1.4 增加 FDCAN2 错误计数诊断通道
 */

/* Includes ------------------------------------------------------------------*/

#include "Chassis.h"
#include "dvc_erictool.h"
#include "fdcan.h"
#include "sys_debug.h"
#include "usb_device.h"
#include "user_task.h"

#include <cmath>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace
{
/** 遥测周期，单位 ms；四轮反馈本身由电机以 100 Hz 主动返回。 */
constexpr uint32_t TELEMETRY_PERIOD_MS = 50U;

/**
 * justfloat 通道数，通道含义固定为：
 *   1~4   左前/右前/左后/右后轮位置，单位度（电机 0x36 反馈，已应用极性）
 *   5~8   左前/右前/左后/右后轮转速，单位 RPM（由位置差分得到）
 *   9~12  左前/右前/左后/右后轮反馈帧计数（判断每台电机是否在回报）
 *   13    FDCAN2 发送错误计数 TEC（0~255，越大越接近总线关闭）
 *   14    FDCAN2 接收错误计数 REC（0~127）
 *   15    FDCAN2 协议状态 PSR：bit0~2 LEC 最近错误码，bit6 EP 被动错误，
 *         bit7 EW 错误警告，bit8 BO 总线关闭
 *   16    FDCAN2 中断标志低 4 位：bit0 RF0N 新报文、bit1 RF0W 水位、
 *         bit2 RF0F FIFO 满、bit3 RF0L 报文丢失（非 0 说明接收溢过）
 */
constexpr int TELEMETRY_CHANNEL_COUNT = 16;

/** 角度差折算到 [-180, 180) 度，避免编码器回绕被当成反向大转角。 */
float WrapAngleDeg(float angle_deg)
{
    angle_deg = std::fmod(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg - 180.0f;
}

/** EricTool 的 Set_Data() 只保存变量地址，遥测镜像必须使用静态存储。 */
float Telemetry_Wheel_Position_Deg[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Speed_Rpm[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Rx_Count[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Can_Tx_Error_Count = 0.0f;
float Telemetry_Can_Rx_Error_Count = 0.0f;
float Telemetry_Can_Protocol_Status = 0.0f;
float Telemetry_Can_Rx_Fifo0_Flags = 0.0f;
float Last_Position_Deg[CHASSIS_WHEEL_COUNT] = {0.0f};
uint64_t Last_Timestamp_Us[CHASSIS_WHEEL_COUNT] = {0U};

/**
 * @brief 取一次四轮反馈并刷新遥测镜像。
 * @note  轮速用本轮的反馈时间戳对位置做差分；该轮没有新反馈帧时保持上一次的值，
 *        此时看 9~12 通道的帧计数不再增加即可判断电机是否掉线。
 */
void UpdateWheelTelemetry()
{
    Struct_Chassis_Wheel_Feedback feedback{};
    APP_Chassis.GetWheelFeedback(feedback);

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        Telemetry_Wheel_Position_Deg[i] = feedback.position_deg[i];
        Telemetry_Wheel_Rx_Count[i] = static_cast<float>(feedback.rx_count[i]);

        const uint64_t now_us = feedback.timestamp_us[i];
        if ((Last_Timestamp_Us[i] != 0U) && (now_us > Last_Timestamp_Us[i]))
        {
            const float dt_s =
                static_cast<float>(now_us - Last_Timestamp_Us[i]) * 1.0e-6f;
            const float delta_deg =
                WrapAngleDeg(feedback.position_deg[i] - Last_Position_Deg[i]);
            // 度每秒换算成 RPM：delta_deg / 360 / dt_s * 60。
            Telemetry_Wheel_Speed_Rpm[i] = delta_deg / (6.0f * dt_s);
        }

        if (now_us != 0U)
        {
            Last_Timestamp_Us[i] = now_us;
            Last_Position_Deg[i] = feedback.position_deg[i];
        }
    }
}

/**
 * @brief 刷新 FDCAN2 的错误计数和协议状态。
 * @note  只读寄存器，用来区分“电机没发报文”和“收发器/总线出错收不到报文”。
 *        TEC/REC 长期为 0 且 PSR 的 BO/EP 位为 0，说明总线本身是健康的。
 */
void UpdateCanTelemetry()
{
    FDCAN_ErrorCountersTypeDef error_counters{};
    if (HAL_FDCAN_GetErrorCounters(&hfdcan2, &error_counters) == HAL_OK)
    {
        Telemetry_Can_Tx_Error_Count = static_cast<float>(error_counters.TxErrorCnt);
        Telemetry_Can_Rx_Error_Count = static_cast<float>(error_counters.RxErrorCnt);
    }

    Telemetry_Can_Protocol_Status =
        static_cast<float>(hfdcan2.Instance->PSR & 0x000001FFU);
    Telemetry_Can_Rx_Fifo0_Flags =
        static_cast<float>(hfdcan2.Instance->IR & 0x0000000FU);
}

/** @brief 把遥测镜像登记到 justfloat 通道，只需在循环外调用一次。 */
template <typename Tool>
void RegisterTelemetryChannels(Tool &tool)
{
    tool.Set_Data(TELEMETRY_CHANNEL_COUNT,
                  reinterpret_cast<int>(&Telemetry_Wheel_Position_Deg[0]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Position_Deg[1]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Position_Deg[2]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Position_Deg[3]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Speed_Rpm[0]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Speed_Rpm[1]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Speed_Rpm[2]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Speed_Rpm[3]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Rx_Count[0]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Rx_Count[1]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Rx_Count[2]),
                  reinterpret_cast<int>(&Telemetry_Wheel_Rx_Count[3]),
                  reinterpret_cast<int>(&Telemetry_Can_Tx_Error_Count),
                  reinterpret_cast<int>(&Telemetry_Can_Rx_Error_Count),
                  reinterpret_cast<int>(&Telemetry_Can_Protocol_Status),
                  reinterpret_cast<int>(&Telemetry_Can_Rx_Fifo0_Flags));
}
} // namespace

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();

    // 变量地址登记一次即可，之后每周期只刷新变量内容。
    RegisterTelemetryChannels(EricTool_USB);
    RegisterTelemetryChannels(EricTool_UART);

    for (;;)
    {
        UpdateWheelTelemetry();
        UpdateCanTelemetry();

        // 两路 justfloat 上行：USB CDC 与 UART7，按实际接线用其中一路即可。
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback();

        osDelay(TELEMETRY_PERIOD_MS);
    }
}
