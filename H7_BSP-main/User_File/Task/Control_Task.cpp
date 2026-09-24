/**
 * @file Control_Task.cpp
 * @brief 四台 ZDT 电机以 50 RPM 驱动底盘向前运行 1 秒。
 */

#include "user_task.h"
#include "Chassis.h"
#include "INS.h"

#include <cstdint>

namespace
{
constexpr float FORWARD_DURATION_S = 1.0f;
constexpr uint16_t ZDT_ACCELERATION = 10U;
constexpr uint32_t ZDT_PULSES_PER_REVOLUTION = 3200U;
constexpr uint32_t ZDT_BOOT_DELAY_TICKS = 2000U;
constexpr uint32_t ZDT_ENABLE_SETTLE_TICKS = 200U;
constexpr float CHASSIS_WHEEL_RADIUS_M = 0.05f;
constexpr float CHASSIS_GEAR_RATIO = 1.0f;

void InitChassis()
{
    Struct_Chassis_Config chassis_config{};

    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
    const int8_t motor_polarity[CHASSIS_WHEEL_COUNT] = {
        1,  // 1 号电机：左前轮，正向
        -1, // 2 号电机：右前轮，反向
        1,  // 3 号电机：左后轮，正向
        -1, // 4 号电机：右后轮，反向
    };

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor_config[i].hfdcan = &hfdcan2;
        motor_config[i].can_id = static_cast<uint8_t>(i + 1U);
        motor_config[i].polarity = motor_polarity[i];
        motor_config[i].speed_to_motor = 1.0f;
        motor_config[i].firmware = Enum_ZDTMotor_Firmware::EMM;
        motor_config[i].checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
        motor_config[i].pulses_per_revolution = ZDT_PULSES_PER_REVOLUTION;
        motor_config[i].acceleration = ZDT_ACCELERATION;
    }

    APP_Chassis.Init(chassis_config, motor_config);

    /* INS 平移只使用轮式里程计，BMI088 在函数 2 中仅提供 Yaw 融合。 */
    Struct_INS_Config ins_config{};
    ins_config.half_length_m = chassis_config.half_length;
    ins_config.half_width_m = chassis_config.half_width;
    ins_config.wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    ins_config.gear_ratio = CHASSIS_GEAR_RATIO;
    APP_INS.Init(ins_config);
}

[[noreturn]] void HoldTask()
{
    for (;;)
    {
        osDelay(1000U);
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
    APP_Chassis.RunForSeconds(30.0f,
                              50.0f,
                              0.0f,
                             3.0f);
       //APP_Chassis.Disable();                       
    HoldTask();
}
