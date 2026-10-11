/**
 * @file Control_Task.cpp
 * @brief 初始化底盘并接收手柄无线控制帧；底盘速度映射留作后续配置。
 */

#include "Com.h"
#include "Chassis.h"
#include "INS.h"
#include "fdcan.h"
#include "cmsis_os2.h"

#include <cstdint>

namespace
{
constexpr uint16_t ZDT_ACCELERATION = 100U;
constexpr uint32_t ZDT_PULSES_PER_REVOLUTION = 3200U;
constexpr uint32_t ZDT_BOOT_DELAY_TICKS = 1000U;
constexpr uint32_t ZDT_ENABLE_SETTLE_TICKS = 1000U;
constexpr uint32_t REMOTE_FRAME_TIMEOUT_TICKS = 100U;
constexpr float CHASSIS_WHEEL_RADIUS_M = 0.0375f;
constexpr float CHASSIS_GEAR_RATIO = 1.0f;

void InitChassis(void)
{
    Struct_Chassis_Config chassis_config{};
    chassis_config.wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    chassis_config.gear_ratio = CHASSIS_GEAR_RATIO;

    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
    const int8_t motor_polarity[CHASSIS_WHEEL_COUNT] = {
        1,  // 左前轮正向
        -1, // 右前轮反向
        1,  // 左后轮正向
        -1, // 右后轮反向
    };

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor_config[i].hfdcan = &hfdcan2;
        motor_config[i].can_id = static_cast<uint8_t>(i + 1U);
        motor_config[i].polarity = motor_polarity[i];
        motor_config[i].firmware = Enum_ZDTMotor_Firmware::EMM;
        motor_config[i].checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
        motor_config[i].pulses_per_revolution = ZDT_PULSES_PER_REVOLUTION;
        motor_config[i].acceleration = ZDT_ACCELERATION;
    }

    APP_Chassis.Init(chassis_config, motor_config);

    Struct_INS_Config ins_config{};
    ins_config.half_length_m = chassis_config.half_length;
    ins_config.half_width_m = chassis_config.half_width;
    ins_config.wheel_radius_m = chassis_config.wheel_radius_m;
    ins_config.gear_ratio = chassis_config.gear_ratio;
    APP_INS.Init(ins_config);
}

void ProcessRemoteFrame(const Struct_Remote_Control_Frame &frame,
                        bool &chassis_stopped)
{
    switch (frame.mode)
    {
    case Enum_Remote_Mode::STOP:
        if (!chassis_stopped)
        {
            APP_Chassis.Stop();
            chassis_stopped = true;
        }
        break;

    case Enum_Remote_Mode::SPEED:
        // 原始 left/right/angle speed 已解析；单位映射和小陀螺固定角速度待手动配置。
        chassis_stopped = false;
        break;

    case Enum_Remote_Mode::RA:
        // angle/height 字段已解析；RA 模式的动作映射暂不猜测。
        chassis_stopped = false;
        break;

    default:
        if (!chassis_stopped)
        {
            APP_Chassis.Stop();
            chassis_stopped = true;
        }
        break;
    }
}
} // namespace

extern "C" void Control_Task(void *argument)
{
    (void)argument;
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    osDelay(ZDT_BOOT_DELAY_TICKS);

    InitChassis();
    APP_Chassis.Enable();
    osDelay(ZDT_ENABLE_SETTLE_TICKS);
    APP_Chassis.Stop();
    bool chassis_stopped = true;
    bool received_frame = false;
    uint32_t last_frame_tick = osKernelGetTickCount();

    for (;;)
    {
        Struct_Remote_Control_Frame frame{};
        if (Communication_TakeLatestFrame(&frame))
        {
            received_frame = true;
            last_frame_tick = osKernelGetTickCount();
            ProcessRemoteFrame(frame, chassis_stopped);
        }

        const uint32_t now = osKernelGetTickCount();
        if (received_frame &&
            ((now - last_frame_tick) >= REMOTE_FRAME_TIMEOUT_TICKS))
        {
            if (!chassis_stopped)
            {
                APP_Chassis.Stop();
                chassis_stopped = true;
            }
            received_frame = false;
        }

        osDelay(1U);
    }
}
