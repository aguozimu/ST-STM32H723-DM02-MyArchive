/**
 * @file Control_Task.cpp
 * @brief 初始化底盘并接收无线控制帧，运行 mode 3 航向与底盘速度闭环。
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
constexpr uint16_t ZDT_ACCELERATION = 250U;
constexpr uint32_t ZDT_PULSES_PER_REVOLUTION = 3200U;
constexpr uint32_t ZDT_BOOT_DELAY_TICKS = 1000U;
constexpr uint32_t ZDT_ENABLE_SETTLE_TICKS = 1000U;
constexpr uint32_t REMOTE_FRAME_TIMEOUT_TICKS = 300U;
constexpr uint32_t REMOTE_CONTROL_PERIOD_TICKS = 10U;
constexpr bool REMOTE_REQUIRE_VALID_INS = true;
constexpr uint64_t CHASSIS_SPEED_FEEDBACK_MAX_AGE_US = 30000U;
constexpr float CHASSIS_WHEEL_RADIUS_M = 0.0375f;
constexpr float CHASSIS_GEAR_RATIO = 1.0f;
constexpr float CHASSIS_MAX_TRANSLATION_SPEED_MPS = 0.75f;
constexpr float CHASSIS_MAX_ANGULAR_SPEED_RAD_S = 2.0f;
constexpr float CHASSIS_MAX_LINEAR_ACCELERATION_MPS2 = 0.5f;
constexpr float CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_S2 = 3.0f;
// 闭环初始值：实机确认方向、单位和响应后再按手感调整。
constexpr float YAW_HOLD_KP_RAD_S_PER_RAD = 2.0f;
constexpr float VELOCITY_PI_KP = 0.5f;
constexpr float VELOCITY_PI_KI_PER_S = 0.6f;
constexpr float VELOCITY_PI_INTEGRAL_LIMIT_MPS = 0.2f;
constexpr float OMEGA_PI_KP = 0.5f;
constexpr float OMEGA_PI_KI_PER_S = 0.6f;
constexpr float OMEGA_PI_INTEGRAL_LIMIT_RAD_S = 0.5f;
constexpr float CONTROL_DT_MAX_S = 0.05f;
constexpr float CONTROL_PI_F = 3.14159265358979f;
// 协议只提供有符号 int8 原始量；按实际手柄标定填写每计数对应的物理速度。
constexpr float REMOTE_X_MPS_PER_COUNT = 0.01f;
constexpr float REMOTE_Y_MPS_PER_COUNT = 0.01f;
constexpr float REMOTE_Z_RAD_S_PER_COUNT = 0.05f;

struct SmallGyroControllerState
{
    bool initialized = false;
    float yaw_target_rad = 0.0f;
    float velocity_integral_world_x = 0.0f;
    float velocity_integral_world_y = 0.0f;
    float omega_integral = 0.0f;
    float command_world_vx_mps = 0.0f;
    float command_world_vy_mps = 0.0f;
    float command_omega_rad_s = 0.0f;

    void Reset()
    {
        *this = SmallGyroControllerState{};
    }
};

float Clamp(float value, float minimum, float maximum)
{
    return (value < minimum) ? minimum : ((value > maximum) ? maximum : value);
}

float NormalizeAngle(float angle_rad)
{
    constexpr float two_pi = 2.0f * CONTROL_PI_F;
    angle_rad = std::fmod(angle_rad + CONTROL_PI_F, two_pi);
    if (angle_rad < 0.0f)
    {
        angle_rad += two_pi;
    }
    return angle_rad - CONTROL_PI_F;
}

void LimitVectorMagnitude(float &x, float &y, float maximum_magnitude)
{
    const float magnitude = std::sqrt(x * x + y * y);
    if (magnitude > maximum_magnitude)
    {
        const float scale = maximum_magnitude / magnitude;
        x *= scale;
        y *= scale;
    }
}

void SlewLimitVector(float target_x,
                     float target_y,
                     float maximum_delta,
                     float &current_x,
                     float &current_y)
{
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;
    const float delta_magnitude = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    if (delta_magnitude > maximum_delta)
    {
        const float scale = maximum_delta / delta_magnitude;
        delta_x *= scale;
        delta_y *= scale;
    }
    current_x += delta_x;
    current_y += delta_y;
}

float UpdateVelocityPI(float target,
                       float measured,
                       float dt_s,
                       float proportional_gain,
                       float integral_gain,
                       float integral_limit,
                       float output_limit,
                       float &integral)
{
    const float error = target - measured;
    const float candidate_integral =
        Clamp(integral + error * integral_gain * dt_s,
              -integral_limit,
              integral_limit);
    float output = target + proportional_gain * error + candidate_integral;

    // 输出饱和且误差仍推动饱和时冻结积分，避免松开手柄后积分残留。
    if ((output > output_limit && error > 0.0f) ||
        (output < -output_limit && error < 0.0f))
    {
        output = target + proportional_gain * error + integral;
    }
    else
    {
        integral = candidate_integral;
    }

    return Clamp(output, -output_limit, output_limit);
}

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

bool ApplySmallGyroFrame(const Struct_Remote_Control_Frame &frame,
                         float dt_s,
                         SmallGyroControllerState &controller)
{
    if (!std::isfinite(dt_s) || !(dt_s > 0.0f) || dt_s > CONTROL_DT_MAX_S)
    {
        return false;
    }

    const Struct_INS_Pose world_pose = APP_INS.GetComplementaryPosition();
    const bool ins_valid =
        world_pose.valid && std::isfinite(world_pose.yaw_rad);
    wireless_ins_valid = ins_valid ? 1U : 0U;
    if (REMOTE_REQUIRE_VALID_INS && !ins_valid)
    {
        return false;
    }
    const float yaw_rad = NormalizeAngle(world_pose.yaw_rad);

    Struct_Chassis_Velocity measured_body_velocity{};
    if (!APP_Chassis.GetBodyVelocity(measured_body_velocity,
                                     CHASSIS_SPEED_FEEDBACK_MAX_AGE_US))
    {
        return false;
    }

    if (!controller.initialized)
    {
        controller.yaw_target_rad = yaw_rad;
        controller.initialized = true;
    }

    float target_world_vx =
        static_cast<float>(frame.x_velocity_raw) * REMOTE_X_MPS_PER_COUNT;
    float target_world_vy =
        static_cast<float>(frame.y_velocity_raw) * REMOTE_Y_MPS_PER_COUNT;
    const float omega_feedforward = Clamp(
        static_cast<float>(frame.z_angular_velocity_raw) *
            REMOTE_Z_RAD_S_PER_COUNT,
        -CHASSIS_MAX_ANGULAR_SPEED_RAD_S,
        CHASSIS_MAX_ANGULAR_SPEED_RAD_S);
    if (!std::isfinite(target_world_vx) || !std::isfinite(target_world_vy) ||
        !std::isfinite(omega_feedforward))
    {
        return false;
    }

    LimitVectorMagnitude(target_world_vx,
                         target_world_vy,
                         CHASSIS_MAX_TRANSLATION_SPEED_MPS);

    // 按手柄角速度积分期望航向；航向误差叠加到角速度目标形成外环。
    controller.yaw_target_rad = NormalizeAngle(
        controller.yaw_target_rad + omega_feedforward * dt_s);
    const float yaw_error_rad =
        NormalizeAngle(controller.yaw_target_rad - yaw_rad);
    const float target_omega = Clamp(
        omega_feedforward + YAW_HOLD_KP_RAD_S_PER_RAD * yaw_error_rad,
        -CHASSIS_MAX_ANGULAR_SPEED_RAD_S,
        CHASSIS_MAX_ANGULAR_SPEED_RAD_S);

    // 将四轮实测车体系速度旋到世界系，在与手柄相同的坐标系做 vx/vy PI。
    const float cos_yaw = std::cos(yaw_rad);
    const float sin_yaw = std::sin(yaw_rad);
    const float measured_world_vx =
        cos_yaw * measured_body_velocity.vx_mps -
        sin_yaw * measured_body_velocity.vy_mps;
    const float measured_world_vy =
        sin_yaw * measured_body_velocity.vx_mps +
        cos_yaw * measured_body_velocity.vy_mps;
    float command_world_vx = UpdateVelocityPI(
        target_world_vx,
        measured_world_vx,
        dt_s,
        VELOCITY_PI_KP,
        VELOCITY_PI_KI_PER_S,
        VELOCITY_PI_INTEGRAL_LIMIT_MPS,
        CHASSIS_MAX_TRANSLATION_SPEED_MPS,
        controller.velocity_integral_world_x);
    float command_world_vy = UpdateVelocityPI(
        target_world_vy,
        measured_world_vy,
        dt_s,
        VELOCITY_PI_KP,
        VELOCITY_PI_KI_PER_S,
        VELOCITY_PI_INTEGRAL_LIMIT_MPS,
        CHASSIS_MAX_TRANSLATION_SPEED_MPS,
        controller.velocity_integral_world_y);
    LimitVectorMagnitude(command_world_vx,
                         command_world_vy,
                         CHASSIS_MAX_TRANSLATION_SPEED_MPS);

    float command_omega = UpdateVelocityPI(
        target_omega,
        measured_body_velocity.omega_rad_s,
        dt_s,
        OMEGA_PI_KP,
        OMEGA_PI_KI_PER_S,
        OMEGA_PI_INTEGRAL_LIMIT_RAD_S,
        CHASSIS_MAX_ANGULAR_SPEED_RAD_S,
        controller.omega_integral);

    // 世界系平移命令做向量加速度限制，减少突加速引起的轮胎打滑。
    SlewLimitVector(command_world_vx,
                    command_world_vy,
                    CHASSIS_MAX_LINEAR_ACCELERATION_MPS2 * dt_s,
                    controller.command_world_vx_mps,
                    controller.command_world_vy_mps);
    const float max_omega_delta = CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_S2 * dt_s;
    command_omega = Clamp(command_omega,
                          controller.command_omega_rad_s - max_omega_delta,
                          controller.command_omega_rad_s + max_omega_delta);
    controller.command_omega_rad_s = command_omega;

    // 将限加速度后的世界系速度旋回车体系，再调用底盘速度入口。
    const float body_vx = cos_yaw * controller.command_world_vx_mps +
                          sin_yaw * controller.command_world_vy_mps;
    const float body_vy = -sin_yaw * controller.command_world_vx_mps +
                          cos_yaw * controller.command_world_vy_mps;
    APP_Chassis.SetVelocity(body_vx, body_vy, command_omega);
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
    SmallGyroControllerState small_gyro_controller{};
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
                if (!small_gyro_active)
                {
                    small_gyro_controller.Reset();
                    last_control_tick = last_frame_tick;
                }
                small_gyro_frame = frame;
                small_gyro_active = true;
            }
            else
            {
                // mode 0/1/2 和未识别模式均按停止处理。
                small_gyro_active = false;
                small_gyro_controller.Reset();
                last_control_tick = last_frame_tick;
                StopChassis(chassis_stopped);
            }
        }

        const uint32_t now = osKernelGetTickCount();
        if (small_gyro_active &&
            ((now - last_frame_tick) >= REMOTE_FRAME_TIMEOUT_TICKS))
        {
            small_gyro_active = false;
            small_gyro_controller.Reset();
            StopChassis(chassis_stopped);
        }

        if (small_gyro_active &&
            ((now - last_control_tick) >= REMOTE_CONTROL_PERIOD_TICKS))
        {
            const uint32_t elapsed_ticks = now - last_control_tick;
            last_control_tick = now;
            const uint32_t tick_frequency = osKernelGetTickFreq();
            const float dt_s = (tick_frequency > 0U)
                                   ? static_cast<float>(elapsed_ticks) /
                                         static_cast<float>(tick_frequency)
                                   : 0.01f;
            if (ApplySmallGyroFrame(small_gyro_frame,
                                    dt_s,
                                    small_gyro_controller))
            {
                chassis_stopped = false;
            }
            else
            {
                // INS 航向或四轮速度反馈无效/超时，不执行开环速度命令。
                small_gyro_active = false;
                small_gyro_controller.Reset();
                StopChassis(chassis_stopped);
            }
        }
        else if (!small_gyro_active)
        {
            last_control_tick = now;
        }

        wireless_control_active = small_gyro_active ? 1U : 0U;

        osDelay(1U);
    }
}
