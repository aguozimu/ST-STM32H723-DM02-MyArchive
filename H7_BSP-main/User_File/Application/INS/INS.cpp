/**
 * @file INS.cpp
 * @brief 陀螺仪 Yaw 与四轮里程计的最简互补融合实现。
 *
 * 融合只用两个输入：四轮位置反馈的增量做麦轮正运动学，BMI088 的 z 轴角速度
 * 做航向增量。X/Y 只由轮式里程计提供，BMI088 仅参与 Yaw 融合，不做加速度
 * 二次积分、静止零速修正、打滑检测或数据有效性判断。
 */

#include "INS.h"

#include "bsp_bmi088.h"
#include "sys_timestamp.h"

#include <cmath>

namespace
{
constexpr float PI_F = 3.14159265358979f;
constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr float ROTATION_RADIUS_EPSILON = 1.0e-6f;
constexpr float MAX_GYRO_DT_S = 0.1f;

/** 把角度折算到 [-pi, pi) 弧度。 */
float WrapAngle(float angle_rad)
{
    const float two_pi = 2.0f * PI_F;
    angle_rad = std::fmod(angle_rad + PI_F, two_pi);
    if (angle_rad < 0.0f)
    {
        angle_rad += two_pi;
    }
    return angle_rad - PI_F;
}

/** 把角度差折算到 [-180, 180) 度，兼容单圈和多圈位置反馈。 */
float WrapAngleDeg(float angle_deg)
{
    angle_deg = std::fmod(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg - 180.0f;
}

float ClampUnit(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

/** 用车体平移增量和航向增量积分二维位姿。 */
void IntegratePose(Struct_INS_Pose &pose,
                   const float delta_x_body_m,
                   const float delta_y_body_m,
                   const float delta_yaw_rad)
{
    const float yaw_mid = WrapAngle(pose.yaw_rad + 0.5f * delta_yaw_rad);
    const float cosine = std::cos(yaw_mid);
    const float sine = std::sin(yaw_mid);

    pose.x_m += cosine * delta_x_body_m - sine * delta_y_body_m;
    pose.y_m += sine * delta_x_body_m + cosine * delta_y_body_m;
    pose.yaw_rad = WrapAngle(pose.yaw_rad + delta_yaw_rad);
    pose.valid = true;
}

} // namespace

Class_INS APP_INS;

void Class_INS::Init(const Struct_INS_Config &new_config)
{
    initialized.store(false, std::memory_order_release);

    /* 互斥锁只创建一次：三组位姿和校准数据都由它保护。 */
    if (pose_mutex == nullptr)
    {
        pose_mutex = osMutexNew(nullptr);
    }

    const Class_Lock_Guard lock(*this);

    config.half_length_m =
        (std::isfinite(new_config.half_length_m) && new_config.half_length_m >= 0.0f)
            ? new_config.half_length_m
            : 0.15f;
    config.half_width_m =
        (std::isfinite(new_config.half_width_m) && new_config.half_width_m >= 0.0f)
            ? new_config.half_width_m
            : 0.15f;
    config.wheel_radius_m =
        (std::isfinite(new_config.wheel_radius_m) && new_config.wheel_radius_m > 0.0f)
            ? new_config.wheel_radius_m
            : 0.05f;
    config.gear_ratio =
        (std::isfinite(new_config.gear_ratio) && new_config.gear_ratio > 0.0f)
            ? new_config.gear_ratio
            : 1.0f;
    config.gyro_weight = std::isfinite(new_config.gyro_weight)
                             ? ClampUnit(new_config.gyro_weight)
                             : INS_DEFAULT_GYRO_WEIGHT;

    ResetState();

    /* 位置反馈和 INS 使用同一个 10 ms 节拍。Chassis::Init() 已配置过一次，
     * 这里显式再配置一次，使 INS 可以作为独立周期模块被重新初始化。 */
    APP_Chassis.ConfigurePeriodicFeedback(INS_UPDATE_PERIOD_MS);
    initialized.store(true, std::memory_order_release);
}

void Class_INS::Reset()
{
    const Class_Lock_Guard lock(*this);
    ResetState();
}

void Class_INS::ResetState()
{
    wheel_position = {};
    gyro_position = {};
    complementary_position = {};

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        last_wheel_position_deg[i] = 0.0f;
        last_wheel_rx_count[i] = 0U;
        wheel_reference_ready[i] = false;
        wheel_pending_ready[i] = false;
        wheel_pending_m[i] = 0.0f;
    }

    wheel_delta_sequence = 0U;
    wheel_delta_x_m = 0.0f;
    wheel_delta_y_m = 0.0f;
    wheel_delta_yaw_rad = 0.0f;

    last_gyro_timestamp_us = 0U;
    gyro_delta_sequence = 0U;
    gyro_delta_yaw_rad = 0.0f;
    gyro_used_wheel_sequence = 0U;
    complementary_used_wheel_sequence = 0U;
    complementary_used_gyro_sequence = 0U;
}

void Class_INS::Lock() const
{
    if (pose_mutex != nullptr)
    {
        (void)osMutexAcquire(pose_mutex, osWaitForever);
    }
}

void Class_INS::Unlock() const
{
    if (pose_mutex != nullptr)
    {
        (void)osMutexRelease(pose_mutex);
    }
}

bool Class_INS::StartCalibration()
{
    const Class_Lock_Guard lock(*this);

    /* 函数 3 互补滤波位姿是底盘使用的世界系位姿，尚未有效时不记录。 */
    if (!complementary_position.valid)
    {
        return false;
    }

    /* 记录当前世界坐标系相关的全部数据：函数 1/2/3 的三组位姿快照。 */
    calibration.start_state.wheel = wheel_position;
    calibration.start_state.gyro = gyro_position;
    calibration.start_state.complementary = complementary_position;
    calibration.active = true;
    return true;
}

bool Class_INS::EndCalibration()
{
    const Class_Lock_Guard lock(*this);

    if (!calibration.active)
    {
        return false;
    }

    /* 本次运行产生的误差：当前世界系位姿 - 校准开始时记录的世界系位姿。 */
    const Struct_INS_Pose &start_pose = calibration.start_state.complementary;
    const float error_x_m = complementary_position.x_m - start_pose.x_m;
    const float error_y_m = complementary_position.y_m - start_pose.y_m;
    const float error_yaw_rad =
        WrapAngle(complementary_position.yaw_rad - start_pose.yaw_rad);

    if (!std::isfinite(error_x_m) || !std::isfinite(error_y_m) ||
        !std::isfinite(error_yaw_rad))
    {
        calibration.active = false;
        return false;
    }

    /* 三组位姿同处一个世界系，统一扣除同一误差，等价于把世界系原点从
     * 校准起点平移到校准终点，即用本次运行的累计误差校准原点位置。 */
    wheel_position.x_m -= error_x_m;
    wheel_position.y_m -= error_y_m;
    wheel_position.yaw_rad = WrapAngle(wheel_position.yaw_rad - error_yaw_rad);

    gyro_position.x_m -= error_x_m;
    gyro_position.y_m -= error_y_m;
    gyro_position.yaw_rad = WrapAngle(gyro_position.yaw_rad - error_yaw_rad);

    complementary_position.x_m -= error_x_m;
    complementary_position.y_m -= error_y_m;
    complementary_position.yaw_rad =
        WrapAngle(complementary_position.yaw_rad - error_yaw_rad);

    calibration.active = false;
    return true;
}

void Class_INS::UpdateWheelPosition()
{
    const Class_Lock_Guard lock(*this);

    if (!initialized.load(std::memory_order_acquire))
    {
        return;
    }

    /* 每轮独立建立位置基准；未收到 0x36 反馈前不参与正运动学。 */
    bool all_wheels_ready = true;
    bool all_wheels_fresh = true;

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const Struct_ZDTMotor_Feedback feedback = APP_Chassis.GetMotorFeedback(i);
        if (feedback.rx_count == 0U)
        {
            all_wheels_ready = false;
            all_wheels_fresh = false;
            continue;
        }

        if (!wheel_reference_ready[i])
        {
            last_wheel_position_deg[i] = feedback.position_deg;
            last_wheel_rx_count[i] = feedback.rx_count;
            wheel_reference_ready[i] = true;
            all_wheels_fresh = false;
            continue;
        }

        /* rx_count 变化表示出现新反馈；位置差先做角度包装，再换算成轮缘线位移。 */
        if (feedback.rx_count != last_wheel_rx_count[i])
        {
            const float delta_deg =
                WrapAngleDeg(feedback.position_deg - last_wheel_position_deg[i]);
            last_wheel_position_deg[i] = feedback.position_deg;
            last_wheel_rx_count[i] = feedback.rx_count;

            wheel_pending_m[i] +=
                delta_deg * DEG_TO_RAD * config.wheel_radius_m / config.gear_ratio;
            wheel_pending_ready[i] = true;
        }

        if (!wheel_reference_ready[i] || !wheel_pending_ready[i])
        {
            all_wheels_fresh = false;
        }
    }

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if (!wheel_reference_ready[i])
        {
            all_wheels_ready = false;
        }
    }

    /* 四轮没有全部到齐时只累计位移，等待下一次调用；避免四轮时间错位造成重复积分。 */
    if (!all_wheels_ready || !all_wheels_fresh)
    {
        return;
    }

    const float ds_left_front = wheel_pending_m[Chassis_Wheel_LeftFront];
    const float ds_right_front = wheel_pending_m[Chassis_Wheel_RightFront];
    const float ds_left_rear = wheel_pending_m[Chassis_Wheel_LeftRear];
    const float ds_right_rear = wheel_pending_m[Chassis_Wheel_RightRear];

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        wheel_pending_m[i] = 0.0f;
        wheel_pending_ready[i] = false;
    }

    /* 与 Class_Chassis::SetVelocity() 的逆运动学严格对称。 */
    wheel_delta_x_m =
        (ds_left_front - ds_right_front - ds_left_rear + ds_right_rear) * 0.25f;
    wheel_delta_y_m =
        (ds_left_front + ds_right_front + ds_left_rear + ds_right_rear) * 0.25f;

    const float rotation_radius = config.half_length_m + config.half_width_m;
    wheel_delta_yaw_rad =
        rotation_radius > ROTATION_RADIUS_EPSILON
            ? (-ds_left_front + ds_right_front - ds_left_rear + ds_right_rear) /
                  (4.0f * rotation_radius)
            : 0.0f;

    /* 仅轮式计数增量写入轮式位姿；函数 2、3 的 X/Y 也来自这组轮式增量。 */
    IntegratePose(wheel_position,
                  wheel_delta_x_m,
                  wheel_delta_y_m,
                  wheel_delta_yaw_rad);
    ++wheel_delta_sequence;
}

void Class_INS::UpdateGyroPosition()
{
    const Class_Lock_Guard lock(*this);

    if (!initialized.load(std::memory_order_acquire))
    {
        return;
    }

    /* 用实际微秒时间戳计算积分间隔，不假定任务永远严格等于 10 ms。 */
    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    if (last_gyro_timestamp_us == 0U)
    {
        last_gyro_timestamp_us = now_us;
        return;
    }

    const uint64_t elapsed_us = now_us - last_gyro_timestamp_us;
    last_gyro_timestamp_us = now_us;
    const float dt_s = static_cast<float>(elapsed_us) * 1.0e-6f;
    if ((dt_s <= 0.0f) || (dt_s > MAX_GYRO_DT_S))
    {
        return;
    }

    /* BMI088 只读取机体系 Z 轴角速度；加速度计不参与 INS 的 X/Y 或 Yaw 融合。 */
    const float gyro_z_rad_s = BSP_BMI088.Get_Gyro_Body().Data[2];
    if (!std::isfinite(gyro_z_rad_s))
    {
        return;
    }

    gyro_delta_yaw_rad = gyro_z_rad_s * dt_s;
    ++gyro_delta_sequence;

    /* 函数 2 的 X/Y 沿用同一周期轮式平移，只用陀螺仪航向旋转，不进行加速度积分。 */
    if (wheel_delta_sequence != gyro_used_wheel_sequence)
    {
        const float yaw_mid =
            WrapAngle(gyro_position.yaw_rad + 0.5f * gyro_delta_yaw_rad);
        const float cosine = std::cos(yaw_mid);
        const float sine = std::sin(yaw_mid);
        gyro_position.x_m +=
            cosine * wheel_delta_x_m - sine * wheel_delta_y_m;
        gyro_position.y_m +=
            sine * wheel_delta_x_m + cosine * wheel_delta_y_m;
        gyro_used_wheel_sequence = wheel_delta_sequence;
    }

    /* Yaw 只由陀螺仪角速度积分；X/Y 不会使用加速度计二次积分。 */
    gyro_position.yaw_rad = WrapAngle(gyro_position.yaw_rad + gyro_delta_yaw_rad);
    gyro_position.valid = true;
}

void Class_INS::UpdateComplementaryPosition()
{
    const Class_Lock_Guard lock(*this);

    if (!initialized.load(std::memory_order_acquire))
    {
        return;
    }

    /* 使用序号区分新轮式增量和陀螺仪增量，保证同一帧只参与互补滤波一次。 */
    const bool has_wheel_delta =
        wheel_delta_sequence != complementary_used_wheel_sequence;
    const bool has_gyro_delta =
        gyro_delta_sequence != complementary_used_gyro_sequence;

    if (!has_wheel_delta && !has_gyro_delta)
    {
        return;
    }

    /* Yaw 为陀螺仪与轮式航向增量的互补滤波；X/Y 始终取轮式正运动学增量。 */
    float delta_yaw_rad = 0.0f;
    if (has_wheel_delta && has_gyro_delta)
    {
        delta_yaw_rad = config.gyro_weight * gyro_delta_yaw_rad +
                        (1.0f - config.gyro_weight) * wheel_delta_yaw_rad;
    }
    else if (has_gyro_delta)
    {
        delta_yaw_rad = gyro_delta_yaw_rad;
    }
    else
    {
        delta_yaw_rad = wheel_delta_yaw_rad;
    }

    /* BMI088 不做平移积分，所以函数 3 的 X/Y 仍完全来自轮式增量。 */
    const float delta_x_body_m = has_wheel_delta ? wheel_delta_x_m : 0.0f;
    const float delta_y_body_m = has_wheel_delta ? wheel_delta_y_m : 0.0f;

    IntegratePose(complementary_position,
                  delta_x_body_m,
                  delta_y_body_m,
                  delta_yaw_rad);

    if (has_wheel_delta)
    {
        complementary_used_wheel_sequence = wheel_delta_sequence;
    }
    if (has_gyro_delta)
    {
        complementary_used_gyro_sequence = gyro_delta_sequence;
    }
}

Struct_INS_Pose Class_INS::GetWheelPosition() const
{
    const Class_Lock_Guard lock(*this);
    return wheel_position;
}

Struct_INS_Pose Class_INS::GetGyroPosition() const
{
    const Class_Lock_Guard lock(*this);
    return gyro_position;
}

Struct_INS_Pose Class_INS::GetComplementaryPosition() const
{
    const Class_Lock_Guard lock(*this);
    return complementary_position;
}

Struct_INS_State Class_INS::GetState() const
{
    const Class_Lock_Guard lock(*this);

    Struct_INS_State state{};
    state.wheel = wheel_position;
    state.gyro = gyro_position;
    state.complementary = complementary_position;
    return state;
}

/** TIM1msTask 10 ms 回调：函数 1，更新轮式编码器位姿。 */
extern "C" void INS_TIM_10ms_UpdateWheelPosition_PeriodElapsedCallback(void)
{
    APP_INS.UpdateWheelPosition();
}

/** TIM1msTask 10 ms 回调：函数 2，更新陀螺仪 Yaw 航向位姿。 */
extern "C" void INS_TIM_10ms_UpdateGyroPosition_PeriodElapsedCallback(void)
{
    APP_INS.UpdateGyroPosition();
}

/** TIM1msTask 10 ms 回调：函数 3，更新互补滤波位姿。 */
extern "C" void INS_TIM_10ms_UpdateComplementaryPosition_PeriodElapsedCallback(void)
{
    APP_INS.UpdateComplementaryPosition();
}
