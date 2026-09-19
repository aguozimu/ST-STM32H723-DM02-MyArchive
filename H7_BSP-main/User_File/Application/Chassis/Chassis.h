/**
 * @file Chassis.h
 * @brief 麦克纳姆轮速度解算与 ZDT 电机调用接口。
 *
 * 本层只保存四台电机的配置、计算麦克纳姆轮目标速度，并转调 ZDTmotor。
 * 参数合法性、命令发送结果和故障状态均不在本层判断。
 */

#ifndef USER_FILE_APPLICATION_CHASSIS_CHASSIS_H
#define USER_FILE_APPLICATION_CHASSIS_CHASSIS_H

#include "zdtmotor.h"

#include <stdint.h>

/** 麦克纳姆底盘固定使用四个车轮。 */
static constexpr uint8_t CHASSIS_WHEEL_COUNT = 4U;

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
    float half_length = 0.15f; //!< 底盘中心到前/后轮轴的距离，通常单位 m。
    float half_width = 0.15f;  //!< 底盘中心到左/右轮的距离，通常单位 m。
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
    float speed_to_motor = 1.0f;          //!< 麦轮解算速度到电机 RPM 的换算系数。
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
     * 四轮速度分别乘以 speed_to_motor 后，以 RPM 传给 ZDTmotor。
     */
    void SetVelocity(float x, float y, float omega);

    /**
     * @brief 按指定速度运行一段时间，然后同步停止四台电机。
     * @param x 向右为正的横向速度。
     * @param y 向前为正的纵向速度。
     * @param omega 逆时针为正的底盘角速度。
     * @param duration_s 运行时长，单位 s，使用 float 表示。
     *
     * 本接口使用 FreeRTOS 的 osDelay() 阻塞当前任务。
     */
    void RunForSeconds(float x, float y, float omega, float duration_s);

    /** 周期调用四台 ZDT 电机的 Process()，维护底层应答和在线状态。 */
    void Process();

    /** 为四台电机缓存停止命令，再发送一次 ZDT 广播同步触发。 */
    void Stop();

    /** 为四台电机缓存失能命令，再发送一次 ZDT 广播同步触发。 */
    void Disable();

private:
    Struct_Chassis_Config config{}; //!< 底盘尺寸配置副本。
    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{}; //!< 四轮电机配置副本。
    Class_ZDTMotor motors[CHASSIS_WHEEL_COUNT]; //!< 左前、右前、左后、右后电机对象。
};

#endif
