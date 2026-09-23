/**
 * @file Control_Task.cpp
 * @brief 初始化麦轮底盘，打开四轮位置反馈，并执行相对位姿闭环示例。
 */

#include "user_task.h"

#include "Chassis.h"
#include "INS.h"

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
/** 等 InsTask 更新几拍，确保 RunToRelativePose() 能读到有效位姿。 */
constexpr uint32_t INS_READY_SETTLE_TICKS = 200U;

/** 验收动作参数：沿车头方向走 1 m、向右横移 0.5 m、原地逆时针转 90 度。 */
constexpr float DEMO_FORWARD_DISTANCE_M = 1.0f;
constexpr float DEMO_SIDEWAY_DISTANCE_M = 0.5f;
constexpr float DEMO_TURN_ANGLE_RAD = 1.5707963f;

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

    // 四轮开始周期返回 0x36 实时位置，INS 才有里程计输入。
    APP_Chassis.ConfigureOdometryFeedback(ODOMETRY_FEEDBACK_PERIOD_MS);
    APP_Chassis.Enable();

    osDelay(ZDT_ENABLE_SETTLE_TICKS);
    osDelay(INS_READY_SETTLE_TICKS);
    //APP_Chassis.RunForSeconds(0.2f, 0.0f, 0.0f, 3);
    // 相对位姿闭环验收动作：沿当前车头前进、向右横移、原地逆时针转 90 度。
    APP_Chassis.RunToRelativePose(0.0f, 0.5f, 0.0f, DEMO_FORWARD_DISTANCE_M);
    //APP_Chassis.RunToRelativePose(0.5f, 0.0f, 0.0f, DEMO_SIDEWAY_DISTANCE_M);
   // APP_Chassis.RunToRelativePose(0.0f, 0.0f, DEMO_TURN_ANGLE_RAD, 0.0f);

    HoldTask();
}
