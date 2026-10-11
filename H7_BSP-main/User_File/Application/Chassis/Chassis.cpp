/**
 * @file Chassis.cpp
 * @brief 麦克纳姆轮速度解算与 ZDT 电机控制实现。
 *
 * 本文件不建立错误状态或检查底层返回值，只负责运动学计算和接口转调。
 */

#include "Chassis.h"

#include "INS.h"
#include "sys_timestamp.h"

#include <cmath>
#include "cmsis_os2.h"

Class_Chassis APP_Chassis;

namespace
{
constexpr float CHASSIS_PI_F = 3.14159265358979f;
constexpr uint32_t POSITION_CONTROL_PERIOD_MS = 100U;
/** Class_PID 的 D_T 必须与控制周期一致，否则微分项按错误时间尺度放大。 */
constexpr float POSITION_CONTROL_PERIOD_S =
    static_cast<float>(POSITION_CONTROL_PERIOD_MS) / 1000.0f;
/** 平移 Kp，1.0 与旧纯 P 的误差即速度行为一致。 */
constexpr float POSITION_PID_KP = 1.5f;
/** 航向 Kp。 */
constexpr float POSITION_PID_KP_YAW = 1.5f;
/** 三轴 Kd 起步值，对测量速度做阻尼；实机过冲加大、发抖减小。 */
constexpr float POSITION_PID_KD = 0.25f;
/** 平移输出限幅 m/s，替代纯 P 时代误差直接当速度的地板油。 */
constexpr float POSITION_PID_OUT_MAX_XY = 0.75f;
/** 航向输出限幅 rad/s。 */
constexpr float POSITION_PID_OUT_MAX_YAW = 1.0f;

float NormalizeAngle(float angle_rad)
{
    constexpr float two_pi = 2.0f * CHASSIS_PI_F;
    angle_rad = std::fmod(angle_rad + CHASSIS_PI_F, two_pi);
    if (angle_rad < 0.0f)
    {
        angle_rad += two_pi;
    }
    return angle_rad - CHASSIS_PI_F;
}
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

    // 位置环三轴 PD：微分先行对测量值微分，目标切换瞬间不产生微分毛刺。
    // 参数与状态封装在 Class_PID 内部，这里只做一次性 Init。
    position_pid_x.Init(POSITION_PID_KP, 0.0f, POSITION_PID_KD, 0.0f, 0.0f,
                        POSITION_PID_OUT_MAX_XY, POSITION_CONTROL_PERIOD_S,
                        0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    position_pid_y.Init(POSITION_PID_KP, 0.0f, POSITION_PID_KD, 0.0f, 0.0f,
                        POSITION_PID_OUT_MAX_XY, POSITION_CONTROL_PERIOD_S,
                        0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    position_pid_yaw.Init(POSITION_PID_KP_YAW, 0.0f, POSITION_PID_KD, 0.0f,
                          0.0f, POSITION_PID_OUT_MAX_YAW, POSITION_CONTROL_PERIOD_S,
                          0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    initialized.store(true, std::memory_order_release);
}

void Class_Chassis::ResetPositionPidState(float x_m, float y_m, float yaw_rad)
{
    // 副作用：清零积分并让三轴 Pre_Now 落到当前位姿——空跑一拍目标=当前、
    // 输出丢弃。不清的话第一拍微分会把位置当速度，出发瞬间输出毛刺。
    position_pid_x.Set_Target(x_m);
    position_pid_x.Set_Now(x_m);
    position_pid_x.Set_Integral_Error(0.0f);
    position_pid_x.TIM_Calculate_PeriodElapsedCallback();

    position_pid_y.Set_Target(y_m);
    position_pid_y.Set_Now(y_m);
    position_pid_y.Set_Integral_Error(0.0f);
    position_pid_y.TIM_Calculate_PeriodElapsedCallback();

    position_pid_yaw.Set_Target(yaw_rad);
    position_pid_yaw.Set_Now(yaw_rad);
    position_pid_yaw.Set_Integral_Error(0.0f);
    position_pid_yaw.TIM_Calculate_PeriodElapsedCallback();

    yaw_pid_last_raw = yaw_rad;
    yaw_pid_unwrapped = yaw_rad;
}

void Class_Chassis::ConfigurePeriodicFeedback(uint16_t period_ms)
{
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].ConfigureSpeedFeedbackPeriod(period_ms);
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

bool Class_Chassis::GetBodyVelocity(Struct_Chassis_Velocity &velocity,
                                   uint64_t max_age_us) const
{
    velocity = {};
    if (!initialized.load(std::memory_order_acquire))
    {
        return false;
    }

    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    float wheel_speed_mps[CHASSIS_WHEEL_COUNT]{};
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const Struct_ZDTMotor_Feedback feedback = motors[i].GetFeedback();
        const uint64_t speed_timestamp_us = feedback.speed_rx_timestamp_us;
        if (!feedback.online || speed_timestamp_us == 0U ||
            speed_timestamp_us > now_us ||
            (now_us - speed_timestamp_us) > max_age_us ||
            !std::isfinite(feedback.speed_rpm) ||
            !std::isfinite(motor_config[i].speed_to_motor) ||
            !(motor_config[i].speed_to_motor > 0.0f))
        {
            return false;
        }

        // speed_to_motor 的单位是 RPM/(m/s)，反馈方向已由 ZDT reverse 统一。
        wheel_speed_mps[i] =
            feedback.speed_rpm / motor_config[i].speed_to_motor;
    }

    const float rotation_radius_m = config.half_length + config.half_width;
    if (!std::isfinite(rotation_radius_m) || !(rotation_radius_m > 0.0f))
    {
        return false;
    }

    // 与 SetVelocity() 的四轮逆运动学严格互逆。
    velocity.vx_mps = (wheel_speed_mps[0] - wheel_speed_mps[1] -
                       wheel_speed_mps[2] + wheel_speed_mps[3]) * 0.25f;
    velocity.vy_mps = (wheel_speed_mps[0] + wheel_speed_mps[1] +
                       wheel_speed_mps[2] + wheel_speed_mps[3]) * 0.25f;
    velocity.omega_rad_s = (-wheel_speed_mps[0] + wheel_speed_mps[1] -
                            wheel_speed_mps[2] + wheel_speed_mps[3]) /
                           (4.0f * rotation_radius_m);

    if (!std::isfinite(velocity.vx_mps) || !std::isfinite(velocity.vy_mps) ||
        !std::isfinite(velocity.omega_rad_s))
    {
        velocity = {};
        return false;
    }

    return true;
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

    // 将解算速度换算为 RPM，并用手册 5.3.1 的一条 AA 多电机命令同时下发。
    float motor_speed_rpm[CHASSIS_WHEEL_COUNT] = {};
    uint16_t motor_acceleration[CHASSIS_WHEEL_COUNT] = {};
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor_speed_rpm[i] = wheel_speed[i] * motor_config[i].speed_to_motor;
        motor_acceleration[i] = motor_config[i].acceleration;
    }
    motors[Chassis_Wheel_LeftFront].SetMultiMotorSpeeds(
        motors, motor_speed_rpm, motor_acceleration, CHASSIS_WHEEL_COUNT);
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

    // 循环外预热三轴 PID：INS 未就绪时快速失败，就绪则清掉上一段的微分残留。
    const Struct_INS_Pose start_position = APP_INS.GetComplementaryPosition();
    if (!start_position.valid || !std::isfinite(start_position.x_m) ||
        !std::isfinite(start_position.y_m) || !std::isfinite(start_position.yaw_rad))
    {
        Stop();
        return false;
    }
    ResetPositionPidState(start_position.x_m,
                          start_position.y_m,
                          start_position.yaw_rad);

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
        const float error_yaw_rad =
            NormalizeAngle(target_yaw_rad - current_position.yaw_rad);
        if ((error_x_m * error_x_m + error_y_m * error_y_m <= tolerance_m * tolerance_m) &&
            (std::fabs(error_yaw_rad) <= yaw_tolerance_rad))
        {
            Stop();
            return true;
        }

        // yaw 解缠：按短弧增量累积连续角，防止过 ±π 时微分项跳变 2π。
        yaw_pid_unwrapped +=
            NormalizeAngle(current_position.yaw_rad - yaw_pid_last_raw);
        yaw_pid_last_raw = current_position.yaw_rad;

        // 三轴 PD：x/y 在世界系算，目标固定；微分先行使 D 项只反映实测速度。
        position_pid_x.Set_Target(target_x_m);
        position_pid_x.Set_Now(current_position.x_m);
        position_pid_x.TIM_Calculate_PeriodElapsedCallback();

        position_pid_y.Set_Target(target_y_m);
        position_pid_y.Set_Now(current_position.y_m);
        position_pid_y.TIM_Calculate_PeriodElapsedCallback();

        // yaw 目标随解缠角平移，PID 内部 Target-Now 恒等于短弧误差。
        position_pid_yaw.Set_Now(yaw_pid_unwrapped);
        position_pid_yaw.Set_Target(yaw_pid_unwrapped + error_yaw_rad);
        position_pid_yaw.TIM_Calculate_PeriodElapsedCallback();

        // 世界系速度指令旋转到车体系后下发，与旧纯 P 的旋转方式一致。
        const float cos_yaw = std::cos(current_position.yaw_rad);
        const float sin_yaw = std::sin(current_position.yaw_rad);
        const float out_x = position_pid_x.Get_Out();
        const float out_y = position_pid_y.Get_Out();
        const float body_vx = cos_yaw * out_x + sin_yaw * out_y;
        const float body_vy = -sin_yaw * out_x + cos_yaw * out_y;
        SetVelocity(body_vx, body_vy, position_pid_yaw.Get_Out());
        osDelay(POSITION_CONTROL_PERIOD_MS);
    }
}

bool Class_Chassis::RunToRelativePosition(float relative_x_m,
                                          float relative_y_m,
                                          float tolerance_m,
                                          float relative_yaw_rad,
                                          float yaw_tolerance_rad)
{
    // 只在进入函数时读取一次当前位置，作为整个相对移动的固定原点。
    const Struct_INS_Pose origin_position = APP_INS.GetComplementaryPosition();

    if (!std::isfinite(relative_x_m) || !std::isfinite(relative_y_m) ||
        !std::isfinite(tolerance_m) || (tolerance_m < 0.0f) ||
        !std::isfinite(relative_yaw_rad) || !std::isfinite(yaw_tolerance_rad) ||
        (yaw_tolerance_rad < 0.0f) ||
        !origin_position.valid || !std::isfinite(origin_position.x_m) ||
        !std::isfinite(origin_position.y_m) ||
        !std::isfinite(origin_position.yaw_rad))
    {
        Stop();
        return false;
    }

    const float cos_origin = std::cos(origin_position.yaw_rad);
    const float sin_origin = std::sin(origin_position.yaw_rad);
    const float target_x_m =
        origin_position.x_m + cos_origin * relative_x_m - sin_origin * relative_y_m;
    const float target_y_m =
        origin_position.y_m + sin_origin * relative_x_m + cos_origin * relative_y_m;
    const float target_yaw_rad = origin_position.yaw_rad + relative_yaw_rad;
    if (!std::isfinite(target_x_m) || !std::isfinite(target_y_m) ||
        !std::isfinite(target_yaw_rad))
    {
        Stop();
        return false;
    }

    // 相对目标换算成世界系绝对目标后直接转发，控制环与 PD 参数只维护一份。
    return RunToPosition(target_x_m,
                         target_y_m,
                         tolerance_m,
                         target_yaw_rad,
                         yaw_tolerance_rad);
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
