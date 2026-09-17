#include "user_task.h"
#include "zdtmotor.h"

namespace
{

Class_ZDTMotor ZDT_Motor;

constexpr uint8_t ZDT_MOTOR_ADDRESS = 1U;
constexpr uint32_t ZDT_MOTOR_PULSES_PER_REVOLUTION = 3200U;
constexpr uint16_t ZDT_SPEED_ACCELERATION = 10U;
constexpr uint16_t ZDT_SPEED_MAX_RPM = 300U;
constexpr uint32_t ZDT_CONTROL_PERIOD_TICKS = 1000U;

} // namespace

extern "C" void Control_Task(void *argument)
{
    (void)argument;

    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    if (!ZDT_Motor.Init(&hfdcan2,
                        ZDT_MOTOR_ADDRESS,
                        Enum_ZDTMotor_Firmware::EMM,
                        Enum_ZDTMotor_Checksum::FIXED_6B,
                        false,
                        ZDT_MOTOR_PULSES_PER_REVOLUTION))
    {
        for (;;)
        {
            osDelay(ZDT_CONTROL_PERIOD_TICKS);
        }
    }

    ZDT_Motor.Enable();
    ZDT_Motor.SetSpeed(0.0f, ZDT_SPEED_ACCELERATION);

    uint16_t target_speed_rpm = 0U;
    uint32_t next_wake_tick =
        osKernelGetTickCount() + ZDT_CONTROL_PERIOD_TICKS;

    for (;;)
    {
        ZDT_Motor.Process();

        target_speed_rpm =
            target_speed_rpm >= ZDT_SPEED_MAX_RPM
                ? 0U
                : static_cast<uint16_t>(target_speed_rpm + 1U);

        ZDT_Motor.SetSpeed(static_cast<float>(target_speed_rpm),
                           ZDT_SPEED_ACCELERATION);

        next_wake_tick += ZDT_CONTROL_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}
