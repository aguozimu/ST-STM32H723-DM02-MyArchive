/**
 * @file Control_Task.cpp
 * @brief 初始化底盘并接收手柄无线控制帧；底盘速度映射留作后续配置。
 */

#include "Com.h"
#include "Chassis.h"
#include "INS.h"
#include "fdcan.h"
#include "cmsis_os2.h"

#include <cmath>
#include <cstdint>

extern "C" {
extern volatile uint8_t wireless_control_active;
extern volatile uint8_t wireless_ins_valid;
}

namespace
{
constexpr uint16_t ZDT_ACCELERATION = 100U;
constexpr uint32_t ZDT_PULSES_PER_REVOLUTION = 3200U;
constexpr uint32_t ZDT_BOOT_DELAY_TICKS = 1000U;
constexpr uint32_t ZDT_ENABLE_SETTLE_TICKS = 1000U;
constexpr uint32_t REMOTE_FRAME_TIMEOUT_TICKS = 100U;
constexpr uint32_t REMOTE_CONTROL_PERIOD_TICKS = 10U;
// Debug fallback: allow mode=3 control before the INS pose becomes valid.
constexpr bool REMOTE_REQUIRE_VALID_INS = false;
constexpr float CHASSIS_WHEEL_RADIUS_M = 0.0375f;
constexpr float CHASSIS_GEAR_RATIO = 1.0f;
// 协议只提供有符号 int8 原始量；按实际手柄标定填写每计数对应的物理速度。
// 比例未配置前保持 0，避免把原始字节误当成 m/s 或 rad/s 直接驱动车轮。
constexpr float REMOTE_X_MPS_PER_COUNT = 0.005f;
constexpr float REMOTE_Y_MPS_PER_COUNT = 0.005f;
constexpr float REMOTE_Z_RAD_S_PER_COUNT = 0.01f;

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

void StopChassis(bool &chassis_stopped)
{
    if (!chassis_stopped)
    {
        APP_Chassis.Stop();
    }
    chassis_stopped = true;
}

bool ApplySmallGyroFrame(const Struct_Remote_Control_Frame &frame)
{
    const Struct_INS_Pose world_pose = APP_INS.GetComplementaryPosition();
    const bool ins_valid =
        world_pose.valid && std::isfinite(world_pose.yaw_rad);
    wireless_ins_valid = ins_valid ? 1U : 0U;
    if (REMOTE_REQUIRE_VALID_INS && !ins_valid)
    {
        return false;
    }
    const float yaw_rad = ins_valid ? world_pose.yaw_rad : 0.0f;

    const float world_vx =
        static_cast<float>(frame.x_velocity_raw) * REMOTE_X_MPS_PER_COUNT;
    const float world_vy =
        static_cast<float>(frame.y_velocity_raw) * REMOTE_Y_MPS_PER_COUNT;
    const float omega = static_cast<float>(frame.z_angular_velocity_raw) *
                        REMOTE_Z_RAD_S_PER_COUNT;

    // 与 Class_Chassis::RunToPosition() 使用相同的世界系到车体系变换。
    const float cos_yaw = std::cos(yaw_rad);
    const float sin_yaw = std::sin(yaw_rad);
    const float body_vx = cos_yaw * world_vx + sin_yaw * world_vy;
    const float body_vy = -sin_yaw * world_vx + cos_yaw * world_vy;
    APP_Chassis.SetVelocity(body_vx, body_vy, omega);
    return true;
}
} // namespace

extern "C" {
volatile uint8_t wireless_control_active = 0U;
volatile uint8_t wireless_ins_valid = 0U;
}

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
    bool small_gyro_active = false;
    Struct_Remote_Control_Frame small_gyro_frame{};
    uint32_t last_frame_tick = osKernelGetTickCount();
    uint32_t last_control_tick = last_frame_tick;

    for (;;)
    {
        Struct_Remote_Control_Frame frame{};
        if (Communication_TakeLatestFrame(&frame))
        {
            last_frame_tick = osKernelGetTickCount();
            if (frame.mode == Enum_Remote_Mode::SMALL_GYRO)
            {
                small_gyro_frame = frame;
                small_gyro_active = true;
            }
            else
            {
                // mode 0/1/2 和未识别模式均按停止处理。
                small_gyro_active = false;
                StopChassis(chassis_stopped);
            }
        }

        const uint32_t now = osKernelGetTickCount();
        if (small_gyro_active &&
            ((now - last_frame_tick) >= REMOTE_FRAME_TIMEOUT_TICKS))
        {
            small_gyro_active = false;
            StopChassis(chassis_stopped);
        }

        if (small_gyro_active &&
            ((now - last_control_tick) >= REMOTE_CONTROL_PERIOD_TICKS))
        {
            last_control_tick = now;
            if (ApplySmallGyroFrame(small_gyro_frame))
            {
                chassis_stopped = false;
            }
            else
            {
                // INS 航向尚未有效时不执行世界系速度命令。
                small_gyro_active = false;
                StopChassis(chassis_stopped);
            }
        }

        wireless_control_active = small_gyro_active ? 1U : 0U;

        osDelay(1U);
    }
}
