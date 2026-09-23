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
#include "INS.h"
#include "bsp_power.h"
#include "dvc_erictool.h"
#include "dvc_teleplot.h"
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
/** 遥测周期，单位 ms；四轮 0x36 位置回包本身为 100 Hz。 */
constexpr uint32_t TELEMETRY_PERIOD_MS = 40U;
/** Teleplot ASCII 波形周期，单位 ms；带读取诊断后按 115200 留足发送时间。 */
constexpr uint32_t TELEPLOT_PERIOD_MS = 80U;

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
float Telemetry_Wheel_Target_Speed_Rpm[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Rx_Count[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Raw_Rx_Count[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Position_Rx_Count[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Last_Function[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Checksum_Error[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Wheel_Checksum_Sum = 0.0f;
float Telemetry_Wheel_Diff_Speed_Rpm[CHASSIS_WHEEL_COUNT] = {0.0f};
float Telemetry_Can_Tx_Error_Count = 0.0f;
float Telemetry_Can_Rx_Error_Count = 0.0f;
float Telemetry_Can_Protocol_Status = 0.0f;
float Telemetry_Can_Rx_Fifo0_Flags = 0.0f;
float Telemetry_Can_Raw_Rx_Count = 0.0f;
float Telemetry_Can_Raw_Last_Id = 0.0f;
float Telemetry_Can_Raw_Last_Dlc = 0.0f;
float Telemetry_Can_Raw_Last_Function = 0.0f;
float Telemetry_Can_Raw_Read_Error = 0.0f;
float Telemetry_Ins_X_M = 0.0f;
float Telemetry_Ins_Y_M = 0.0f;
float Telemetry_Ins_Yaw_Rad = 0.0f;
float Telemetry_Battery_Voltage = 0.0f;
float Telemetry_Cmd_Speed_M_S = 0.0f;
float Telemetry_Cmd_Direction_Deg = 0.0f;
float Telemetry_Cmd_Yaw_Rate_Rad_S = 0.0f;
float Telemetry_Fbk_Speed_M_S = 0.0f;
float Telemetry_Fbk_Direction_Deg = 0.0f;
float Telemetry_Fbk_Yaw_Rate_Rad_S = 0.0f;
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
    Telemetry_Wheel_Checksum_Sum = 0.0f;

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        Telemetry_Wheel_Position_Deg[i] = feedback.position_deg[i];
        Telemetry_Wheel_Rx_Count[i] = static_cast<float>(feedback.rx_count[i]);
        Telemetry_Wheel_Raw_Rx_Count[i] = static_cast<float>(feedback.raw_rx_count[i]);
        Telemetry_Wheel_Position_Rx_Count[i] =
            static_cast<float>(feedback.position_rx_count[i]);
        Telemetry_Wheel_Last_Function[i] = static_cast<float>(feedback.last_function[i]);
        Telemetry_Wheel_Checksum_Error[i] =
            static_cast<float>(feedback.checksum_error_count[i]);
        Telemetry_Wheel_Checksum_Sum +=
            static_cast<float>(feedback.checksum_error_count[i]);
        if (feedback.speed_rx_count[i] > 0U)
        {
            Telemetry_Wheel_Speed_Rpm[i] = feedback.speed_rpm[i];
        }
        else
        {
            Telemetry_Wheel_Speed_Rpm[i] = Telemetry_Wheel_Diff_Speed_Rpm[i];
        }

        const uint64_t now_us = feedback.timestamp_us[i];
        if ((Last_Timestamp_Us[i] != 0U) && (now_us > Last_Timestamp_Us[i]))
        {
            const float dt_s =
                static_cast<float>(now_us - Last_Timestamp_Us[i]) * 1.0e-6f;
            const float delta_deg =
                WrapAngleDeg(feedback.position_deg[i] - Last_Position_Deg[i]);
            // 度每秒换算成 RPM：delta_deg / 360 / dt_s * 60。
            Telemetry_Wheel_Diff_Speed_Rpm[i] = delta_deg / (6.0f * dt_s);
        }

        if (now_us != 0U)
        {
            Last_Timestamp_Us[i] = now_us;
            Last_Position_Deg[i] = feedback.position_deg[i];
        }
    }

    APP_Chassis.GetWheelTargetSpeedRpm(Telemetry_Wheel_Target_Speed_Rpm);

    Struct_Chassis_Velocity target_velocity{};
    APP_Chassis.GetTargetVelocity(target_velocity);
    Telemetry_Cmd_Speed_M_S =
        std::sqrt(target_velocity.x_m_s * target_velocity.x_m_s +
                  target_velocity.y_m_s * target_velocity.y_m_s);
    Telemetry_Cmd_Direction_Deg =
        Telemetry_Cmd_Speed_M_S > 1.0e-3f
            ? std::atan2(target_velocity.x_m_s, target_velocity.y_m_s) *
                  (180.0f / 3.14159265358979f)
            : 0.0f;
    Telemetry_Cmd_Yaw_Rate_Rad_S = target_velocity.omega_rad_s;

    Struct_Chassis_Velocity feedback_velocity{};
    APP_Chassis.CalculateBodyVelocityFromWheelRpm(Telemetry_Wheel_Speed_Rpm,
                                                   feedback_velocity);
    Telemetry_Fbk_Speed_M_S =
        std::sqrt(feedback_velocity.x_m_s * feedback_velocity.x_m_s +
                  feedback_velocity.y_m_s * feedback_velocity.y_m_s);
    Telemetry_Fbk_Direction_Deg =
        Telemetry_Fbk_Speed_M_S > 1.0e-3f
            ? std::atan2(feedback_velocity.x_m_s, feedback_velocity.y_m_s) *
                  (180.0f / 3.14159265358979f)
            : 0.0f;
    Telemetry_Fbk_Yaw_Rate_Rad_S = feedback_velocity.omega_rad_s;

    const Struct_INS_State ins = APP_INS.GetState();
    Telemetry_Ins_X_M = ins.x_m;
    Telemetry_Ins_Y_M = ins.y_m;
    Telemetry_Ins_Yaw_Rad = ins.yaw_rad;
    Telemetry_Battery_Voltage = BSP_Power.Get_Power_Voltage();
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

    Struct_CAN_Rx_Diagnostic diagnostic{};
    if (BSP_CAN_GetRxDiagnostic(&hfdcan2, &diagnostic))
    {
        Telemetry_Can_Raw_Rx_Count = static_cast<float>(diagnostic.rx_count);
        Telemetry_Can_Raw_Last_Id = static_cast<float>(diagnostic.last_id);
        Telemetry_Can_Raw_Last_Dlc = static_cast<float>(diagnostic.last_length);
        Telemetry_Can_Raw_Last_Function = static_cast<float>(diagnostic.last_data[0]);
        Telemetry_Can_Raw_Read_Error =
            static_cast<float>(diagnostic.rx_read_error_count);
    }
}

[[maybe_unused]] float Teleplot_Sample_SpdCmdLf() { return Telemetry_Wheel_Target_Speed_Rpm[0]; }
[[maybe_unused]] float Teleplot_Sample_SpdCmdRf() { return Telemetry_Wheel_Target_Speed_Rpm[1]; }
[[maybe_unused]] float Teleplot_Sample_SpdCmdLr() { return Telemetry_Wheel_Target_Speed_Rpm[2]; }
[[maybe_unused]] float Teleplot_Sample_SpdCmdRr() { return Telemetry_Wheel_Target_Speed_Rpm[3]; }
[[maybe_unused]] float Teleplot_Sample_SpdFbkLf() { return Telemetry_Wheel_Speed_Rpm[0]; }
[[maybe_unused]] float Teleplot_Sample_SpdFbkRf() { return Telemetry_Wheel_Speed_Rpm[1]; }
[[maybe_unused]] float Teleplot_Sample_SpdFbkLr() { return Telemetry_Wheel_Speed_Rpm[2]; }
[[maybe_unused]] float Teleplot_Sample_SpdFbkRr() { return Telemetry_Wheel_Speed_Rpm[3]; }
[[maybe_unused]] float Teleplot_Sample_PosFbkLf() { return Telemetry_Wheel_Position_Deg[0]; }
[[maybe_unused]] float Teleplot_Sample_PosFbkRf() { return Telemetry_Wheel_Position_Deg[1]; }
[[maybe_unused]] float Teleplot_Sample_PosFbkLr() { return Telemetry_Wheel_Position_Deg[2]; }
[[maybe_unused]] float Teleplot_Sample_PosFbkRr() { return Telemetry_Wheel_Position_Deg[3]; }
[[maybe_unused]] float Teleplot_Sample_ChecksumSum() { return Telemetry_Wheel_Checksum_Sum; }
[[maybe_unused]] float Teleplot_Sample_RawRxLf() { return Telemetry_Wheel_Raw_Rx_Count[0]; }
[[maybe_unused]] float Teleplot_Sample_RawRxRf() { return Telemetry_Wheel_Raw_Rx_Count[1]; }
[[maybe_unused]] float Teleplot_Sample_RawRxLr() { return Telemetry_Wheel_Raw_Rx_Count[2]; }
[[maybe_unused]] float Teleplot_Sample_RawRxRr() { return Telemetry_Wheel_Raw_Rx_Count[3]; }
[[maybe_unused]] float Teleplot_Sample_PosRxLf() { return Telemetry_Wheel_Position_Rx_Count[0]; }
[[maybe_unused]] float Teleplot_Sample_PosRxRf() { return Telemetry_Wheel_Position_Rx_Count[1]; }
[[maybe_unused]] float Teleplot_Sample_PosRxLr() { return Telemetry_Wheel_Position_Rx_Count[2]; }
[[maybe_unused]] float Teleplot_Sample_PosRxRr() { return Telemetry_Wheel_Position_Rx_Count[3]; }
[[maybe_unused]] float Teleplot_Sample_FnLf() { return Telemetry_Wheel_Last_Function[0]; }
[[maybe_unused]] float Teleplot_Sample_FnRf() { return Telemetry_Wheel_Last_Function[1]; }
[[maybe_unused]] float Teleplot_Sample_FnLr() { return Telemetry_Wheel_Last_Function[2]; }
[[maybe_unused]] float Teleplot_Sample_FnRr() { return Telemetry_Wheel_Last_Function[3]; }
[[maybe_unused]] float Teleplot_Sample_CeLf() { return Telemetry_Wheel_Checksum_Error[0]; }
[[maybe_unused]] float Teleplot_Sample_CeRf() { return Telemetry_Wheel_Checksum_Error[1]; }
[[maybe_unused]] float Teleplot_Sample_CeLr() { return Telemetry_Wheel_Checksum_Error[2]; }
[[maybe_unused]] float Teleplot_Sample_CeRr() { return Telemetry_Wheel_Checksum_Error[3]; }
[[maybe_unused]] float Teleplot_Sample_CanRawRx() { return Telemetry_Can_Raw_Rx_Count; }
[[maybe_unused]] float Teleplot_Sample_CanRawId() { return Telemetry_Can_Raw_Last_Id; }
[[maybe_unused]] float Teleplot_Sample_CanRawDlc() { return Telemetry_Can_Raw_Last_Dlc; }
[[maybe_unused]] float Teleplot_Sample_CanRawFn() { return Telemetry_Can_Raw_Last_Function; }
[[maybe_unused]] float Teleplot_Sample_CanReadError() { return Telemetry_Can_Raw_Read_Error; }
[[maybe_unused]] float Teleplot_Sample_CanTec() { return Telemetry_Can_Tx_Error_Count; }
[[maybe_unused]] float Teleplot_Sample_CanRec() { return Telemetry_Can_Rx_Error_Count; }
[[maybe_unused]] float Teleplot_Sample_CanPsr() { return Telemetry_Can_Protocol_Status; }
[[maybe_unused]] float Teleplot_Sample_CanIr() { return Telemetry_Can_Rx_Fifo0_Flags; }
[[maybe_unused]] float Teleplot_Sample_CmdSpeed() { return Telemetry_Cmd_Speed_M_S; }
[[maybe_unused]] float Teleplot_Sample_CmdDirection() { return Telemetry_Cmd_Direction_Deg; }
[[maybe_unused]] float Teleplot_Sample_CmdYawRate() { return Telemetry_Cmd_Yaw_Rate_Rad_S; }
[[maybe_unused]] float Teleplot_Sample_FbkSpeed() { return Telemetry_Fbk_Speed_M_S; }
[[maybe_unused]] float Teleplot_Sample_FbkDirection() { return Telemetry_Fbk_Direction_Deg; }
[[maybe_unused]] float Teleplot_Sample_FbkYawRate() { return Telemetry_Fbk_Yaw_Rate_Rad_S; }
[[maybe_unused]] float Teleplot_Sample_NavX() { return Telemetry_Ins_X_M; }
[[maybe_unused]] float Teleplot_Sample_NavY() { return Telemetry_Ins_Y_M; }
[[maybe_unused]] float Teleplot_Sample_NavYaw() { return Telemetry_Ins_Yaw_Rad; }
[[maybe_unused]] float Teleplot_Sample_BatteryVoltage() { return Telemetry_Battery_Voltage; }

/** @brief 注册 Teleplot 通道；当前只保留运动数据和 0x36/CAN 读取诊断。 */
void RegisterTeleplotChannels()
{
    Teleplot.Channel_Add("spd_lf", Teleplot_Sample_SpdFbkLf);
    Teleplot.Channel_Add("spd_rf", Teleplot_Sample_SpdFbkRf);
    Teleplot.Channel_Add("spd_lr", Teleplot_Sample_SpdFbkLr);
    Teleplot.Channel_Add("spd_rr", Teleplot_Sample_SpdFbkRr);
    Teleplot.Channel_Add("pos_lf", Teleplot_Sample_PosFbkLf);
    Teleplot.Channel_Add("pos_rf", Teleplot_Sample_PosFbkRf);
    Teleplot.Channel_Add("pos_lr", Teleplot_Sample_PosFbkLr);
    Teleplot.Channel_Add("pos_rr", Teleplot_Sample_PosFbkRr);
    Teleplot.Channel_Add("rx_lf", Teleplot_Sample_PosRxLf);
    Teleplot.Channel_Add("rx_rf", Teleplot_Sample_PosRxRf);
    Teleplot.Channel_Add("rx_lr", Teleplot_Sample_PosRxLr);
    Teleplot.Channel_Add("rx_rr", Teleplot_Sample_PosRxRr);
    Teleplot.Channel_Add("can_rx", Teleplot_Sample_CanRawRx);
    Teleplot.Channel_Add("can_id", Teleplot_Sample_CanRawId);
    Teleplot.Channel_Add("can_tec", Teleplot_Sample_CanTec);
    Teleplot.Channel_Add("can_psr", Teleplot_Sample_CanPsr);
    Teleplot.Channel_Add("can_ir", Teleplot_Sample_CanIr);
    Teleplot.Channel_Add("can_rerr", Teleplot_Sample_CanReadError);
    Teleplot.Channel_Add("ce_sum", Teleplot_Sample_ChecksumSum);
    Teleplot.Channel_Add("v_cmd", Teleplot_Sample_CmdSpeed);
    Teleplot.Channel_Add("dir_cmd", Teleplot_Sample_CmdDirection);
    Teleplot.Channel_Add("v_fbk", Teleplot_Sample_FbkSpeed);
    Teleplot.Channel_Add("dir_fbk", Teleplot_Sample_FbkDirection);
    Teleplot.Channel_Add("vbat", Teleplot_Sample_BatteryVoltage);
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

    // UART7 改由 Teleplot ASCII 波形输出，初始化后再打开发送。
    Teleplot.Init(&huart7, TELEPLOT_PERIOD_MS);
    RegisterTeleplotChannels();
    Teleplot.Set_Enabled(true);

    // USB 保留原 justfloat；UART7 原 EricTool 通道注销但不删除。
    RegisterTelemetryChannels(EricTool_USB);
    // RegisterTelemetryChannels(EricTool_UART);

    for (;;)
    {
        UpdateWheelTelemetry();
        UpdateCanTelemetry();

        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        // UART7 原 EricTool justfloat 发送停用，改由 Teleplot 周期任务发送。
        // EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback();

        osDelay(TELEMETRY_PERIOD_MS);
    }
}
