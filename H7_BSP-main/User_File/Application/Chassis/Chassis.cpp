/**
 * @file Chassis.cpp
 * @brief 麦克纳姆轮速度解算与 ZDT 电机控制实现。
 *
 * 本文件负责运动学计算、接口转调和最简距离闭环，不建立错误状态机，也不
 * 根据底层返回值做故障处理。
 */

#include "Chassis.h"

#include "INS.h"

#include "cmsis_os2.h"

#include <cmath>

namespace
{
constexpr float PI_F = 3.14159265358979f;

/** 距离闭环 P 参数：平移和航向分开整定。 */
constexpr float KP_XY = 1.2f;
constexpr float KP_YAW = 2.0f;
/** 到位判据：世界系位置误差和航向误差同时小于阈值才算到位。 */
constexpr float POSITION_TOLERANCE_M = 0.02f;
constexpr float YAW_TOLERANCE_RAD = 0.05f;
/** 距离闭环周期，与 InsTask 的 100 Hz 位姿更新率一致。 */
constexpr uint32_t CONTROL_PERIOD_MS = 10U;
/** 超时退出时间，只用于防止死循环，不是故障检测。 */
constexpr uint32_t TIMEOUT_MS = 60000U;
/** 平移方向的最小模长，低于该值认为方向未定义。 */
constexpr float DIRECTION_EPSILON = 1.0e-3f;

/** 把角度折算到 [-pi, pi) 弧度。 */
float Wrap_Angle(float angle_rad)
{
    const float two_pi = 2.0f * PI_F;
    angle_rad = std::fmod(angle_rad + PI_F, two_pi);
    if (angle_rad < 0.0f)
    {
        angle_rad += two_pi;
    }
    return angle_rad - PI_F;
}

/** 按对称上下限限幅。 */
float Clamp(float value, float limit)
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
} // namespace

Class_Chassis APP_Chassis;

void Class_Chassis::Init(
    const Struct_Chassis_Config &new_config,
    const Struct_Chassis_Motor_Config new_motor_config[CHASSIS_WHEEL_COUNT])
{
    // 保存配置副本，后续 SetVelocity() 直接使用这些参数完成速度换算。
    config = new_config;

    // 麦轮线速度 m/s 换算成电机转速 RPM：v / (2*pi*r) 转每秒，乘 60 得 RPM，
    // 再乘减速比得到电机轴转速。轮径非法时保持 1:1 透传，避免出现除零。
    float speed_to_motor = 1.0f;
    if (config.wheel_radius_m > 0.0f)
    {
        speed_to_motor =
            config.gear_ratio * 60.0f / (2.0f * PI_F * config.wheel_radius_m);
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
    CancelRunTimer();

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

    const double duration_ms_float = static_cast<double>(duration_s) * 1000.0;
    uint32_t duration_ms = UINT32_MAX;
    if (duration_ms_float < static_cast<double>(UINT32_MAX))
    {
        duration_ms = static_cast<uint32_t>(duration_ms_float);
        if (static_cast<double>(duration_ms) < duration_ms_float)
        {
            ++duration_ms;
        }
        if (duration_ms == 0U)
        {
            duration_ms = 1U;
        }
    }

    // SetVelocity() 会先取消旧倒计时，随后用新的时长开始毫秒倒计时。
    SetVelocity(x, y, omega);
    run_remaining_ms.store(duration_ms, std::memory_order_release);
}

void Class_Chassis::TIM_1ms_RunForSeconds_PeriodElapsedCallback()
{
    uint32_t remaining_ms = run_remaining_ms.load(std::memory_order_acquire);
    while (remaining_ms > 0U)
    {
        if (run_remaining_ms.compare_exchange_weak(remaining_ms,
                                                   remaining_ms - 1U,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_acquire))
        {
            if (remaining_ms == 1U)
            {
                Stop();
            }
            return;
        }
    }
}

void Class_Chassis::Process()
{
    // ZDTmotor 在 Process() 中维护运动应答超时与在线状态。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Process();
    }
}

void Class_Chassis::Stop()
{
    CancelRunTimer();

    // 先缓存四条停止命令，再通过一次广播让四轮同时停止。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Stop(true);
    }
    motors[Chassis_Wheel_LeftFront].TriggerSynchronousMotion();
}

void Class_Chassis::Disable()
{
    CancelRunTimer();

    // 先缓存四条失能命令，再通过一次广播让四轮同时失能。
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motors[i].Disable(true);
    }
    motors[Chassis_Wheel_LeftFront].TriggerSynchronousMotion();
}

void Class_Chassis::CancelRunTimer()
{
    run_remaining_ms.store(0U, std::memory_order_release);
}

bool Class_Chassis::ConfigureOdometryFeedback(uint16_t period_ms)
{
    bool ok = true;
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        // 只要有一台电机的配置命令没有入队，就返回 false，但其余电机照常配置。
        ok = motors[i].ConfigurePeriodicFeedback(0x36U, period_ms) && ok;
    }
    return ok;
}

bool Class_Chassis::StopOdometryFeedback()
{
    // 周期写 0 表示停止该功能码的定时返回。
    return ConfigureOdometryFeedback(0U);
}

bool Class_Chassis::GetWheelFeedback(Struct_Chassis_Wheel_Feedback &feedback) const
{
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const Struct_ZDTMotor_Feedback motor_feedback = motors[i].GetFeedback();
        feedback.position_deg[i] = motor_feedback.position_deg;
        feedback.speed_rpm[i] = motor_feedback.speed_rpm;
        feedback.rx_count[i] = motor_feedback.rx_count;
        feedback.timestamp_us[i] = motor_feedback.last_rx_timestamp_us;
    }
    return true;
}

bool Class_Chassis::RunToRelativePose(float x, float y, float omega, float distance_m)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(omega) ||
        !std::isfinite(distance_m) || (distance_m < 0.0f))
    {
        return false;
    }

    const float direction_norm = std::sqrt(x * x + y * y);
    if ((distance_m > 0.0f) && (direction_norm < DIRECTION_EPSILON))
    {
        // 要走一段距离却没有给出平移方向，属于参数非法。
        return false;
    }

    if ((distance_m == 0.0f) && (omega == 0.0f))
    {
        // 既不平移也不转向，空操作，不打扰正在进行的运动。
        return true;
    }

    const Struct_INS_State start = APP_INS.GetState();
    if (!start.valid)
    {
        return false;
    }

    // 方向只取单位向量，模长不参与距离计算。
    float dx_body_target = 0.0f;
    float dy_body_target = 0.0f;
    if (direction_norm > DIRECTION_EPSILON)
    {
        dx_body_target = x / direction_norm * distance_m;
        dy_body_target = y / direction_norm * distance_m;
    }

    // 目标点由调用瞬间的位姿算出，机体系位移转到世界系后叠加到起点上。
    const float start_cos = std::cos(start.yaw_rad);
    const float start_sin = std::sin(start.yaw_rad);
    const float target_x =
        start.x_m + start_cos * dx_body_target - start_sin * dy_body_target;
    const float target_y =
        start.y_m + start_sin * dx_body_target + start_cos * dy_body_target;
    const float target_yaw = Wrap_Angle(start.yaw_rad + omega);

    uint32_t elapsed_ms = 0U;
    while (elapsed_ms < TIMEOUT_MS)
    {
        const Struct_INS_State state = APP_INS.GetState();

        const float ex_world = target_x - state.x_m;
        const float ey_world = target_y - state.y_m;
        const float eyaw = Wrap_Angle(target_yaw - state.yaw_rad);

        if ((std::fabs(ex_world) < POSITION_TOLERANCE_M) &&
            (std::fabs(ey_world) < POSITION_TOLERANCE_M) &&
            (std::fabs(eyaw) < YAW_TOLERANCE_RAD))
        {
            Stop();
            return true;
        }

        // 世界系误差换算到机体系后直接用 P 控制，限幅后交给麦轮逆运动学。
        const float yaw_cos = std::cos(state.yaw_rad);
        const float yaw_sin = std::sin(state.yaw_rad);
        const float ex_body = yaw_cos * ex_world + yaw_sin * ey_world;
        const float ey_body = -yaw_sin * ex_world + yaw_cos * ey_world;

        SetVelocity(Clamp(KP_XY * ex_body, config.max_linear_speed_m_s),
                    Clamp(KP_XY * ey_body, config.max_linear_speed_m_s),
                    Clamp(KP_YAW * eyaw, config.max_angular_speed_rad_s));

        // 阻塞等待期间 InsTask 仍在更新位姿，下一轮用新位姿重新计算误差。
        osDelay(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
    }

    Stop();
    return false;
}

bool Class_Chassis::SetWheelPolarity(Enum_Chassis_Wheel wheel, int8_t polarity)
{
    if ((wheel >= CHASSIS_WHEEL_COUNT) || ((polarity != 1) && (polarity != -1)))
    {
        return false;
    }

    // 只改方向标志，不重新 Init()，避免重复注册同一个 CAN 接收回调。
    Stop();
    motor_config[wheel].polarity = polarity;
    motors[wheel].SetReverse(polarity < 0);
    return true;
}

extern "C" void APP_Chassis_TIM_1ms_RunForSeconds_PeriodElapsedCallback(void)
{
    APP_Chassis.TIM_1ms_RunForSeconds_PeriodElapsedCallback();
}
