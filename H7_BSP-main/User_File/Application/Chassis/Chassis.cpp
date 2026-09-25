/**
 * @file Chassis.cpp
 * @brief 麦克纳姆轮速度解算与 ZDT 电机控制实现。
 *
 * 本文件不建立错误状态或检查底层返回值，只负责运动学计算和接口转调。
 */

#include "Chassis.h"

#include "INS.h"

#include <cmath>
#include "cmsis_os2.h"

Class_Chassis APP_Chassis;

namespace
{
constexpr float CHASSIS_PI_F = 3.14159265358979f;
constexpr uint32_t POSITION_CONTROL_PERIOD_MS = 100U;
} // namespace

void Class_Chassis::Init(
    const Struct_Chassis_Config &new_config,
    const Struct_Chassis_Motor_Config new_motor_config[CHASSIS_WHEEL_COUNT])
{
    // 保存配置副本，后续 SetVelocity() 直接使用这些参数完成速度换算。
    config = new_config;

    // m/s 到电机 RPM：v / (2*pi*r) * 60 * 减速比。
    float speed_to_motor = 1.0f;
    if (std::isfinite(config.wheel_radius_m) && (config.wheel_radius_m > 0.0f) &&
        std::isfinite(config.gear_ratio) && (config.gear_ratio > 0.0f))
    {
        speed_to_motor =
            config.gear_ratio * 60.0f / (2.0f * CHASSIS_PI_F * config.wheel_radius_m);
    }

    // 数组下标与车轮枚举一一对应：左前、右前、左后、右后。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor_config[i] = new_motor_config[i];
        motor_config[i].speed_to_motor = speed_to_motor;

        // ZDTmotor 使用 reverse 表示方向，这里将 +1/-1 极性转换为布尔值。
        motors[i].Init(motor_config[i].hfdcan,
                       motor_config[i].can_id,
                       motor_config[i].firmware,
                       motor_config[i].checksum,
                       motor_config[i].polarity < 0,
                       motor_config[i].pulses_per_revolution);
    }

    ConfigurePeriodicFeedback();
    initialized.store(true, std::memory_order_release);
}

void Class_Chassis::ConfigurePeriodicFeedback(uint16_t period_ms)
{
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        //motors[i].ConfigureSpeedFeedbackPeriod(period_ms);
        motors[i].ConfigurePositionFeedbackPeriod(period_ms);
    }
}

Struct_ZDTMotor_Feedback Class_Chassis::GetMotorFeedback(uint8_t motor_index) const
{
    if (!initialized.load(std::memory_order_acquire) ||
        motor_index >= CHASSIS_WHEEL_COUNT)
    {
        return {};
    }

    return motors[motor_index].GetFeedback();
}

void Class_Chassis::Enable()
{
    // true 表示只将使能命令缓存到各电机，暂不单独执行。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Enable(true);
    }
    // 任意同总线、同校验配置的电机对象都可发送地址 0 的广播触发帧。
    motors[Chassis_Wheel_LeftFront].TriggerSynchronousMotion();
}

void Class_Chassis::SetVelocity(float x, float y, float omega)
{
    // 麦轮旋转项 R*omega，其中 R 为底盘半长与半宽之和。
    const float rotation = (config.half_length + config.half_width) * omega;

    // 逆运动学结果顺序固定为左前、右前、左后、右后。
    const float wheel_speed[CHASSIS_WHEEL_COUNT] = {
        y + x - rotation, // 左前轮
        y - x + rotation, // 右前轮
        y - x - rotation, // 左后轮
        y + x + rotation, // 右后轮
    };

    // 将解算速度换算为 RPM，并把四条速度命令缓存到 ZDT 电机。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].SetSpeed(wheel_speed[i] * motor_config[i].speed_to_motor,
                           motor_config[i].acceleration,
                           true);
    }
    // 四条缓存命令在收到广播帧后同时开始执行。
    motors[Chassis_Wheel_LeftFront].TriggerSynchronousMotion();
}

void Class_Chassis::RunForSeconds(float x,
                                  float y,
                                  float omega,
                                  float duration_s)
{
    if (!(duration_s > 0.0f) || !std::isfinite(duration_s))
    {
        Stop();
        return;
    }

    SetVelocity(x, y, omega);
    osDelay(static_cast<uint32_t>(duration_s * 1000.0f));
    Stop();
}

bool Class_Chassis::RunToPosition(float target_x_m,
                                  float target_y_m,
                                  float tolerance_m,
                                  float target_yaw_rad,
                                  float yaw_tolerance_rad)
{
    if (!std::isfinite(target_x_m) || !std::isfinite(target_y_m) ||
        !std::isfinite(tolerance_m) || (tolerance_m < 0.0f) ||
        !std::isfinite(target_yaw_rad) || !std::isfinite(yaw_tolerance_rad) ||
        (yaw_tolerance_rad < 0.0f))
    {
        Stop();
        return false;
    }

    for (;;)
    {
        const Struct_INS_Pose current_position = APP_INS.GetComplementaryPosition();
        if (!current_position.valid || !std::isfinite(current_position.x_m) ||
            !std::isfinite(current_position.y_m) ||
            !std::isfinite(current_position.yaw_rad))
        {
            Stop();
            return false;
        }

        const float error_x_m = target_x_m - current_position.x_m;
        const float error_y_m = target_y_m - current_position.y_m;
        const float error_yaw_rad = target_yaw_rad - current_position.yaw_rad;
        if ((error_x_m * error_x_m + error_y_m * error_y_m <= tolerance_m * tolerance_m) &&
            (std::fabs(error_yaw_rad) <= yaw_tolerance_rad))
        {
            Stop();
            return true;
        }

        SetVelocity(error_x_m, error_y_m, error_yaw_rad);
        osDelay(POSITION_CONTROL_PERIOD_MS);
    }
}

void Class_Chassis::Process()
{
    if (!initialized.load(std::memory_order_acquire))
    {
        return;
    }

    // ZDTmotor 在 Process() 中维护运动应答超时与在线状态。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Process();
    }
}

void Class_Chassis::Stop()
{
    // 先缓存四条停止命令，再通过一次广播让四轮同时停止。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Stop(true);
    }
    motors[Chassis_Wheel_LeftFront].TriggerSynchronousMotion();
}

void Class_Chassis::Disable()
{
    // 先缓存四条失能命令，再通过一次广播让四轮同时失能。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Disable(true);
    }
    motors[Chassis_Wheel_LeftFront].TriggerSynchronousMotion();
}
