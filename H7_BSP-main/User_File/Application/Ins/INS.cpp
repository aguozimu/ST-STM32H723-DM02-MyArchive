/**
 * @file INS.cpp
 * @brief 陀螺仪与四轮里程计的最简融合实现。
 *
 * 融合只用两个输入：四轮位置反馈的增量做麦轮正运动学，BMI088 的 z 轴角速度
 * 做航向增量，两者按固定权重加权后积分成 x / y / yaw。这里不做 EKF、不做
 * 打滑检测、不做数据有效性判断。
 */

#include "INS.h"

#include "bsp_bmi088.h"
#include "sys_timestamp.h"

#include <cmath>

namespace
{
constexpr float PI_F = 3.14159265358979f;
/** 航向融合权重：越大越信任陀螺仪，越不信任轮式里程计。 */
constexpr float GYRO_WEIGHT = 0.98f;
/** 位置反馈单位为度，换算成弧度用的常数。 */
constexpr float DEG_TO_RAD = PI_F / 180.0f;
/** 旋转半径的最小有效值，避免配置全零时出现除零。 */
constexpr float ROTATION_RADIUS_EPSILON = 1.0e-6f;

/** 把角度差折算到 [-180, 180) 度，等价于反复做 ±360 修正。 */
float WrapAngleDeg(float angle_deg)
{
    angle_deg = std::fmod(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg - 180.0f;
}

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
} // namespace

Class_INS APP_INS;

void Class_INS::Init(const Struct_Chassis_Config &config)
{
    half_length = config.half_length;
    half_width = config.half_width;
    wheel_radius_m = config.wheel_radius_m;
    /* 减速比是除数，配置非法时按直驱处理，避免后续出现除零。 */
    gear_ratio = config.gear_ratio > 0.0f ? config.gear_ratio : 1.0f;

    first_sample = true;
    ResetPose();
}

void Class_INS::ResetPose()
{
    state.x_m = 0.0f;
    state.y_m = 0.0f;
    state.yaw_rad = 0.0f;
    state.valid = false;
    last_timestamp_us = 0U;
}

Struct_INS_State Class_INS::GetState() const
{
    return state;
}

void Class_INS::UpdateVisionPose(const Struct_Vision_Pose_Observation &observation)
{
    vision_pose = observation;
}

Struct_Vision_Pose_Observation Class_INS::GetVisionPose() const
{
    return vision_pose;
}

bool Class_INS::Update()
{
    Struct_Chassis_Wheel_Feedback wheel{};
    APP_Chassis.GetWheelFeedback(wheel);

    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();

    /* 第一帧只建立基准，不产生位移。 */
    if (first_sample || (last_timestamp_us == 0U))
    {
        for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            /* 还没收到过位置反馈的轮子先不建立基准，否则会把电机上电时的
             * 绝对位置误当成一次位移。 */
            wheel_reference_ready[i] = wheel.rx_count[i] > 0U;
            last_position_deg[i] = wheel.position_deg[i];
        }
        last_timestamp_us = now_us;
        first_sample = false;
        state.valid = true;
        return true;
    }

    const float dt = static_cast<float>(now_us - last_timestamp_us) * 1.0e-6f;
    if (dt <= 0.0f)
    {
        return false;
    }
    last_timestamp_us = now_us;

    /* 四轮位移，单位 m；位置反馈已按极性处理，这里不再按左右轮改符号。 */
    float ds[CHASSIS_WHEEL_COUNT] = {0.0f};
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if (!wheel_reference_ready[i])
        {
            /* 本轮第一次拿到位置反馈，本周期只建立基准，不产生位移。 */
            if (wheel.rx_count[i] > 0U)
            {
                last_position_deg[i] = wheel.position_deg[i];
                wheel_reference_ready[i] = true;
            }
            continue;
        }

        const float d_deg =
            WrapAngleDeg(wheel.position_deg[i] - last_position_deg[i]);
        last_position_deg[i] = wheel.position_deg[i];
        ds[i] = d_deg * DEG_TO_RAD / gear_ratio * wheel_radius_m;
    }

    /* 麦轮正运动学，与 Class_Chassis::SetVelocity() 的逆运动学严格对称。 */
    const float ds_left_front = ds[Chassis_Wheel_LeftFront];
    const float ds_right_front = ds[Chassis_Wheel_RightFront];
    const float ds_left_rear = ds[Chassis_Wheel_LeftRear];
    const float ds_right_rear = ds[Chassis_Wheel_RightRear];

    const float dx_body =
        (ds_left_front - ds_right_front - ds_left_rear + ds_right_rear) * 0.25f;
    const float dy_body =
        (ds_left_front + ds_right_front + ds_left_rear + ds_right_rear) * 0.25f;

    const float rotation_radius = half_length + half_width;
    const float dyaw_wheel =
        rotation_radius > ROTATION_RADIUS_EPSILON
            ? (-ds_left_front + ds_right_front - ds_left_rear + ds_right_rear) /
                  (4.0f * rotation_radius)
            : 0.0f;

    /* 陀螺仪 + 轮式航向增量互补融合。 */
    const float gyro_z = BSP_BMI088.Get_Gyro_Body().Data[2];
    const float dyaw =
        GYRO_WEIGHT * (gyro_z * dt) + (1.0f - GYRO_WEIGHT) * dyaw_wheel;

    /* 位置积分用周期中点航向，边走边转时误差更小。 */
    const float yaw_mid = state.yaw_rad + 0.5f * dyaw;
    const float yaw_mid_cos = std::cos(yaw_mid);
    const float yaw_mid_sin = std::sin(yaw_mid);
    state.x_m += yaw_mid_cos * dx_body - yaw_mid_sin * dy_body;
    state.y_m += yaw_mid_sin * dx_body + yaw_mid_cos * dy_body;
    state.yaw_rad = WrapAngle(state.yaw_rad + dyaw);
    state.valid = true;
    return true;
}
