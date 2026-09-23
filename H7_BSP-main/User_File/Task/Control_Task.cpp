/**
 * @file Control_Task.cpp
 * @brief 初始化麦轮底盘，打开四轮位置反馈，并接收 UART7 速度/距离命令。
 */

#include "user_task.h"

#include "Chassis.h"
#include "INS.h"
#include "dvc_serial_command.h"

#include <cmath>
#include <cstdint>

namespace
{
/** 底盘机械参数：麦轮有效直径 75 mm，电机与麦轮直驱。 */
constexpr float CHASSIS_HALF_LENGTH_M = 0.073f;
constexpr float CHASSIS_HALF_WIDTH_M = 0.146f;
constexpr float WHEEL_RADIUS_M = 0.0375f;
constexpr float CHASSIS_GEAR_RATIO = 1.0f;

/** 距离闭环的速度上限，按地面附着力给。 */
constexpr float MAX_LINEAR_SPEED_M_S = 0.30f;
constexpr float MAX_ANGULAR_SPEED_RAD_S = 1.0f;

/** 四轮 0x36 实时位置的主动返回周期。 */
constexpr uint16_t ODOMETRY_FEEDBACK_PERIOD_MS = 10U;
constexpr uint16_t ZDT_ACCELERATION = 10U;
constexpr uint32_t ZDT_PULSES_PER_REVOLUTION = 3200U;
constexpr uint32_t ZDT_BOOT_DELAY_TICKS = 2000U;
constexpr uint32_t ZDT_ENABLE_SETTLE_TICKS = 200U;
/** 等 InsTask 更新几拍，确保距离命令能读到有效位姿。 */
constexpr uint32_t INS_READY_SETTLE_TICKS = 200U;

/** 命令方向角与闭环周期。 */
constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;
constexpr uint32_t CONTROL_PERIOD_MS = 10U;
constexpr uint32_t FEEDBACK_RETRY_PERIOD_MS = 1000U;
constexpr uint32_t POSITION_READ_TEST_PERIOD_MS = 200U;
constexpr float STARTUP_FORWARD_SPEED_M_S = 0.05f;

float Clamp_Symmetric(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void InitChassis()
{
    Struct_Chassis_Config chassis_config{};
    chassis_config.half_length = CHASSIS_HALF_LENGTH_M;
    chassis_config.half_width = CHASSIS_HALF_WIDTH_M;
    chassis_config.wheel_radius_m = WHEEL_RADIUS_M;
    chassis_config.gear_ratio = CHASSIS_GEAR_RATIO;
    chassis_config.max_linear_speed_m_s = MAX_LINEAR_SPEED_M_S;
    chassis_config.max_angular_speed_rad_s = MAX_ANGULAR_SPEED_RAD_S;

    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
    // 地址 1~4 依次对应左前、右前、左后、右后。
    // 极性和滚子方向必须按实车测量结果修改：这里沿用台架初值，1、3 正极性，
    // 2、4 反极性。
    const int8_t motor_polarity[CHASSIS_WHEEL_COUNT] = {
        1,  // 1 号电机：左前轮，正向
        -1, // 2 号电机：右前轮，反向
        1,  // 3 号电机：左后轮，正向
        -1, // 4 号电机：右后轮，反向
    };
    const Enum_Chassis_Roller_Orientation motor_roller[CHASSIS_WHEEL_COUNT] = {
        Chassis_Roller_BackSlash,    // 左前轮
        Chassis_Roller_ForwardSlash, // 右前轮
        Chassis_Roller_ForwardSlash, // 左后轮
        Chassis_Roller_BackSlash,    // 右后轮
    };

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor_config[i].hfdcan = &hfdcan2;
        motor_config[i].can_id = static_cast<uint8_t>(i + 1U);
        motor_config[i].wheel = static_cast<Enum_Chassis_Wheel>(i);
        motor_config[i].roller_orientation = motor_roller[i];
        motor_config[i].polarity = motor_polarity[i];
        motor_config[i].firmware = Enum_ZDTMotor_Firmware::EMM;
        motor_config[i].checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
        motor_config[i].pulses_per_revolution = ZDT_PULSES_PER_REVOLUTION;
        motor_config[i].acceleration = ZDT_ACCELERATION;
    }

    APP_Chassis.Init(chassis_config, motor_config);
    // INS 复用同一份机械参数做轮式里程计换算。
    APP_INS.Init(chassis_config);
}

void Ensure_Realtime_Feedback(uint32_t now_ms)
{
    static uint32_t next_retry_ms = FEEDBACK_RETRY_PERIOD_MS;
    if (now_ms < next_retry_ms)
    {
        return;
    }
    next_retry_ms = now_ms + FEEDBACK_RETRY_PERIOD_MS;

    Struct_Chassis_Wheel_Feedback feedback{};
    APP_Chassis.GetWheelFeedback(feedback);
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if ((feedback.speed_rx_count[i] == 0U) ||
            (feedback.position_rx_count[i] == 0U))
        {
            APP_Chassis.ConfigureRealtimeFeedback(ODOMETRY_FEEDBACK_PERIOD_MS);
            return;
        }
    }
}

void Ensure_Position_Read_Test(uint32_t now_ms)
{
    static uint32_t next_read_ms = POSITION_READ_TEST_PERIOD_MS;
    if (now_ms < next_read_ms)
    {
        return;
    }
    next_read_ms = now_ms + POSITION_READ_TEST_PERIOD_MS;

    Struct_Chassis_Wheel_Feedback feedback{};
    APP_Chassis.GetWheelFeedback(feedback);
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if (feedback.position_rx_count[i] == 0U)
        {
            APP_Chassis.RequestPositionRead();
            return;
        }
    }
}

void Handle_Command(const Struct_SerialCommand &command)
{
    switch (command.type)
    {
    case Enum_SerialCommand_Type::VELOCITY:
    {
        APP_Chassis.CancelRelativePose();
        const float direction_rad = command.direction_deg * DEG_TO_RAD;
        const float speed_m_s = Clamp_Symmetric(command.value, MAX_LINEAR_SPEED_M_S);
        const float omega_rad_s =
            Clamp_Symmetric(command.omega_rad_s, MAX_ANGULAR_SPEED_RAD_S);
        APP_Chassis.SetVelocity(speed_m_s * std::sin(direction_rad),
                                speed_m_s * std::cos(direction_rad),
                                omega_rad_s);
        break;
    }

    case Enum_SerialCommand_Type::DISTANCE:
    {
        const float direction_rad = command.direction_deg * DEG_TO_RAD;
        const float omega_rad_s =
            Clamp_Symmetric(command.omega_rad_s, MAX_ANGULAR_SPEED_RAD_S);
        if (!APP_Chassis.StartRelativePose(std::sin(direction_rad),
                                           std::cos(direction_rad),
                                           omega_rad_s,
                                           command.value))
        {
            APP_Chassis.Stop();
        }
        break;
    }

    case Enum_SerialCommand_Type::STOP:
        APP_Chassis.CancelRelativePose();
        APP_Chassis.Stop();
        break;

    case Enum_SerialCommand_Type::NONE:
    default:
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

    // 先使能并停车，再配置 0x35 实际转速和 0x36 实时位置反馈。
    APP_Chassis.Enable();
    APP_Chassis.Stop();
     APP_Chassis.Disable();
    osDelay(ZDT_ENABLE_SETTLE_TICKS);

    // 四轮同时返回 0x35 转速与 0x36 位置，INS 使用位置，遥测使用实际转速。
    APP_Chassis.ConfigureRealtimeFeedback(ODOMETRY_FEEDBACK_PERIOD_MS);
    osDelay(INS_READY_SETTLE_TICKS);

    // 上电自动测试：整车缓慢向前，后续 UART7 命令仍可覆盖和停车。
    APP_Chassis.SetVelocity(0.0f, STARTUP_FORWARD_SPEED_M_S, 0.0f);
    for (;;)
    {
        Struct_SerialCommand command{};
        while (SerialCommand.Read(command))
        {
            Handle_Command(command);
        }

        const uint32_t now_ms = osKernelGetTickCount();
        Ensure_Position_Read_Test(now_ms);
        Ensure_Realtime_Feedback(now_ms);

        if (APP_Chassis.IsRelativePoseActive())
        {
            APP_Chassis.UpdateRelativePose();
        }

        osDelay(CONTROL_PERIOD_MS);
    }
}
