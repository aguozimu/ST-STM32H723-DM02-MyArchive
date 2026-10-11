/**
 * @file Chassis.h
 * @brief 麦克纳姆轮速度解算与 ZDT 电机调用接口。
 *
 * 本层只保存四台电机的配置、计算麦克纳姆轮目标速度，并转调 ZDTmotor。
 * 参数合法性、命令发送结果和故障状态均不在本层判断。
 */

#ifndef USER_FILE_APPLICATION_CHASSIS_CHASSIS_H
#define USER_FILE_APPLICATION_CHASSIS_CHASSIS_H

#include "alg_pid.h"
#include "zdtmotor.h"

#include <atomic>
#include <stdint.h>

/** 麦克纳姆底盘固定使用四个车轮。 */
static constexpr uint8_t CHASSIS_WHEEL_COUNT = 4U;
/** 四台 ZDT 电机周期反馈默认发送周期，单位 ms。 */
static constexpr uint16_t CHASSIS_MOTOR_FEEDBACK_PERIOD_MS = 10U;

/**
 * @brief 车轮在配置数组、电机数组和速度数组中的固定下标。
 *
 * 所有四元素数组必须按左前、右前、左后、右后的顺序填写。
 */
enum Enum_Chassis_Wheel : uint8_t
{
    Chassis_Wheel_LeftFront = 0U,  //!< 左前轮。
    Chassis_Wheel_RightFront,     //!< 右前轮。
    Chassis_Wheel_LeftRear,       //!< 左后轮。
    Chassis_Wheel_RightRear,      //!< 右后轮。
};

/**
 * @brief 麦克纳姆轮旋转速度解算所需的底盘尺寸。
 *
 * 两个长度必须与 SetVelocity() 的 x、y、omega 所采用的单位体系一致。
 * 常规用法是长度使用 m，x/y 使用 m/s，omega 使用 rad/s。
 */
struct Struct_Chassis_Config
{
    float half_length = 0.075f;    //!< 底盘中心到前/后轮轴的距离，单位 m。
    float half_width = 0.205f;     //!< 底盘中心到左/右轮的距离，单位 m。
    float wheel_radius_m = 0.0375f; //!< 麦轮有效滚动半径，单位 m。
    float gear_ratio = 1.0f;      //!< 电机转数 / 车轮转数，直驱为 1。
};

/** @brief 从四轮实时转速反算的车体系底盘速度。 */
struct Struct_Chassis_Velocity
{
    float vx_mps = 0.0f;       //!< 向右为正的横向速度，单位 m/s。
    float vy_mps = 0.0f;       //!< 向前为正的纵向速度，单位 m/s。
    float omega_rad_s = 0.0f;  //!< 逆时针为正的角速度，单位 rad/s。
};

/**
 * @brief 单个车轮对应的 ZDT 电机参数。
 *
 * 四台电机若使用同步触发，应位于同一 FDCAN 总线并采用相同校验方式；
 * 节点地址应互不相同。这些约束由调用方配置，本层不检查。
 */
struct Struct_Chassis_Motor_Config
{
    FDCAN_HandleTypeDef *hfdcan = nullptr; //!< 电机所在的 FDCAN 外设。
    uint8_t can_id = 0U;                  //!< ZDT 节点地址，地址 0 用于广播。
    int8_t polarity = 1;                  //!< 电机极性：+1 正向，-1 反向。
    float speed_to_motor = 1.0f;          //!< Init() 自动计算的 m/s 到电机 RPM 系数。
    Enum_ZDTMotor_Firmware firmware = Enum_ZDTMotor_Firmware::EMM; //!< ZDT 固件协议。
    Enum_ZDTMotor_Checksum checksum = Enum_ZDTMotor_Checksum::FIXED_6B; //!< 校验方式。
    uint32_t pulses_per_revolution = 3200U; //!< EMM 固件每圈脉冲数。
    uint16_t acceleration = 10U; //!< EMM 为 0~255 档位，X 固件为 RPM/s。
};

/** 麦克纳姆轮速度解算和四台 ZDT 电机的薄封装。 */
class Class_Chassis
{
public:
    /**
     * @brief 保存底盘参数并初始化四台 ZDT 电机。
     *
     * motor_config 的数组顺序必须与 Enum_Chassis_Wheel 一致。
     * ZDTmotor::Init() 的返回值不会在本层处理。
     */
    void Init(const Struct_Chassis_Config &config,
              const Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]);

    /** 为四台电机缓存使能命令，再发送一次 ZDT 广播同步触发。 */
    void Enable();

    /**
     * @brief 计算四个麦克纳姆轮的速度并同步发送给 ZDT 电机。
     * @param x 向右为正的横向速度，通常单位 m/s。
     * @param y 向前为正的纵向速度，通常单位 m/s。
     * @param omega 逆时针为正的底盘角速度，通常单位 rad/s。
     *
     * 四轮速度分别乘以 speed_to_motor 后，以一条 0xAA 多电机命令传给 ZDTmotor。
     */
    void SetVelocity(float x, float y, float omega);

    /**
     * @brief 按指定速度运行一段时间，然后同步停止四台电机。
     * @param x 向右为正的横向速度。
     * @param y 向前为正的纵向速度。
     * @param omega 逆时针为正的底盘角速度。
     * @param duration_s 运行时长，单位 s，使用 float 表示。
     *
     * 本接口使用 osDelay() 阻塞当前任务，延时结束后同步停止四台电机。
     */
    void RunForSeconds(float x, float y, float omega, float duration_s);

    /**
     * @brief 最简单的阻塞式位置移动。
     * @param target_x_m 目标 x 坐标，单位 m，与 Struct_INS_Pose.x_m 一致。
     * @param target_y_m 目标 y 坐标，单位 m，与 Struct_INS_Pose.y_m 一致。
     * @param tolerance_m 到位阈值，单位 m。
     * @param target_yaw_rad 目标 yaw，单位 rad，与 Struct_INS_Pose.yaw_rad 一致。
     * @param yaw_tolerance_rad yaw 到位阈值，单位 rad。
     *
     * 循环读取 APP_INS.GetComplementaryPosition()，x/y/yaw 三轴误差分别经
     * Class_PID 做 PD（微分先行、带输出限幅）得到世界系速度，再旋转到车体系
     * 作为 SetVelocity() 输入。每个循环使用 osDelay(100) 等待，不做加减速规划。
     */
    bool RunToPosition(float target_x_m,
                       float target_y_m,
                       float tolerance_m,
                       float target_yaw_rad = 0.0f,
                       float yaw_tolerance_rad = 0.05f);

    /**
     * @brief 从函数开始时的当前位置出发，相对移动指定距离。
     * @param relative_x_m 相对目标 x 位移，单位 m，向右为正。
     * @param relative_y_m 相对目标 y 位移，单位 m，向前为正。
     * @param tolerance_m 到位阈值，单位 m。
     * @param relative_yaw_rad 相对目标 yaw，单位 rad，逆时针为正。
     * @param yaw_tolerance_rad yaw 到位阈值，单位 rad。
     *
     * 函数开始时读取一次 APP_INS.GetComplementaryPosition() 作为原点，
     * 将相对位移按原点 yaw 旋转到世界系后与原点相加得到目标坐标，随后直接
     * 转发 RunToPosition()，控制环只在那一处维护。
     */
    bool RunToRelativePosition(float relative_x_m,
                               float relative_y_m,
                               float tolerance_m,
                               float relative_yaw_rad = 0.0f,
                               float yaw_tolerance_rad = 0.05f);

    /** 周期调用四台 ZDT 电机的 Process()，维护底层应答和在线状态。 */
    void Process();

    /**
     * @brief 为四台电机配置定时返回；period_ms 为 0 时停止返回。
     *
     * 电机手册 5.5.1 的定时返回命令一次只带一个信息功能码，因此这里只配置 0x36
     * 实时位置（INS 里程计要用，轮速差分也依赖它）。0x35 实时转速不再单独配置，
     * 改由 GetBodyVelocity() 用位置差分推算，避免两个功能码互相顶掉。
     */
    void ConfigurePeriodicFeedback(uint16_t period_ms = CHASSIS_MOTOR_FEEDBACK_PERIOD_MS);

    /** 按车轮顺序读取一台电机的反馈快照；索引越界或尚未初始化时返回默认值。 */
    Struct_ZDTMotor_Feedback GetMotorFeedback(uint8_t motor_index) const;

    /**
     * @brief 反算车体系 vx/vy/角速度。
     * @param velocity 成功时写入反算结果；失败时写零。
     * @param max_age_us 单轮转速允许的最大年龄，单位 us。
     * @return 四个轮速都新鲜时返回 true。
     *
     * 每轮转速优先取 0x35 实时转速；若该功能码没有定时返回（speed_rx_timestamp_us
     * 一直为 0），则用 0x36 实时位置的两拍差分估算，位置与时间戳取自同一条反馈，
     * 因此不会把位置当成速度。任一轮两路都拿不到新鲜数据就返回 false，由调用方停车。
     */
    bool GetBodyVelocity(Struct_Chassis_Velocity &velocity,
                         uint64_t max_age_us = 30000U) const;

    /** 为四台电机缓存停止命令，再发送一次 ZDT 广播同步触发。 */
    void Stop();

    /** 为四台电机缓存失能命令，再发送一次 ZDT 广播同步触发。 */
    void Disable();

private:
    /**
     * @brief 用当前位姿复位三轴位置环 PID 的历史状态。
     * @param x_m 当前 x，单位 m。
     * @param y_m 当前 y，单位 m。
     * @param yaw_rad 当前 yaw，单位 rad。
     *
     * 预热微分基准并清除跨航点残留，避免出发第一拍把位置当速度输出毛刺。
     */
    void ResetPositionPidState(float x_m, float y_m, float yaw_rad);

    Struct_Chassis_Config config{}; //!< 底盘尺寸配置副本。
    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{}; //!< 四轮电机配置副本。
    Class_ZDTMotor motors[CHASSIS_WHEEL_COUNT]; //!< 左前、右前、左后、右后电机对象。

    /** 位置环三轴 PD：x/y 在世界系计算，yaw 用解缠连续角，输出转车体系下发。 */
    Class_PID position_pid_x{};   //!< 世界系 x 位置环。
    Class_PID position_pid_y{};   //!< 世界系 y 位置环。
    Class_PID position_pid_yaw{}; //!< 航向位置环，内部误差为短弧误差。
    float yaw_pid_last_raw = 0.0f;  //!< 上一拍原始 yaw，用于解缠差分。
    float yaw_pid_unwrapped = 0.0f; //!< 连续 yaw，仅喂给 PID 的微分项。

    /** 单轮转速差分的上一拍位置样本。 */
    struct WheelSpeedHistory
    {
        float position_deg = 0.0f;   //!< 上一拍位置，单位 度。
        uint64_t timestamp_us = 0U;  //!< 上一拍位置反馈时间戳，us。
        bool valid = false;          //!< 是否已有可用的上一拍样本。
    };

    /** 单轮最近一次算出的转速样本，位置这一拍没更新时沿用。 */
    struct WheelSpeedSample
    {
        float speed_rpm = 0.0f;      //!< 最近一次算出的轮速，单位 RPM。
        uint64_t timestamp_us = 0U;  //!< 该样本对应的反馈时间戳，us。
        bool valid = false;
    };

    /**
     * @brief 取单轮转速：优先 0x35 实时转速，缺失时用 0x36 位置差分。
     * @return 拿到新鲜转速返回 true；拿不到时由调用方停止底盘。
     *
     * 在本 const 查询里缓存上一拍位置/时间戳与最近转速样本，所以状态声明为 mutable。
     */
    bool ResolveWheelSpeedRpm(uint8_t wheel_index,
                              const Struct_ZDTMotor_Feedback &feedback,
                              uint64_t now_us,
                              uint64_t max_age_us,
                              float &speed_rpm) const;

    mutable WheelSpeedHistory wheel_speed_history[CHASSIS_WHEEL_COUNT]{};
    mutable WheelSpeedSample wheel_speed_sample[CHASSIS_WHEEL_COUNT]{};

    std::atomic<bool> initialized{false}; //!< 四台电机 Init() 完成后才允许任务读取反馈。
};

/** 各任务共用的应用层底盘实例。 */
extern Class_Chassis APP_Chassis;

#endif
