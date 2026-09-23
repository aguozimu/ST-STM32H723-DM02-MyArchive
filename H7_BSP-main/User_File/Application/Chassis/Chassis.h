/**
 * @file Chassis.h
 * @brief 麦克纳姆轮速度解算与 ZDT 电机调用接口。
 *
 * 本层保存四台电机的配置，计算麦克纳姆轮目标速度，转调 ZDTmotor，并在此之上
 * 提供最简单的距离闭环 RunToRelativePose()。
 * 参数合法性只在公开接口的入口做最必要的判断，命令发送结果和故障状态不在本层
 * 处理。
 */

#ifndef USER_FILE_APPLICATION_CHASSIS_CHASSIS_H
#define USER_FILE_APPLICATION_CHASSIS_CHASSIS_H

#include "zdtmotor.h"

#include <atomic>
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
 * @brief 麦轮滚子方向，只用于配置记录和人工核对。
 *
 * 距离闭环用的运动学公式按标准 X 型麦轮构型写死，本字段不参与任何计算。
 */
enum Enum_Chassis_Roller_Orientation : int8_t
{
    Chassis_Roller_ForwardSlash = 1,  //!< 从车顶俯视，接地滚子轴线为斜杠方向。
    Chassis_Roller_BackSlash = -1,    //!< 从车顶俯视，接地滚子轴线为反斜杠方向。
};

/**
 * @brief 麦克纳姆轮旋转速度解算所需的底盘尺寸。
 *
 * 两个长度必须与 SetVelocity() 的 x、y、omega 所采用的单位体系一致。
 * 常规用法是长度使用 m，x/y 使用 m/s，omega 使用 rad/s。
 */
struct Struct_Chassis_Config
{
    float half_length = 0.15f; //!< 底盘中心到前/后轮轴的距离，单位 m。
    float half_width = 0.15f;  //!< 底盘中心到左/右轮接触中心的距离，单位 m。
    float wheel_radius_m = 0.05f; //!< 麦轮有效滚动半径，单位 m。
    float gear_ratio = 1.0f;   //!< 电机转数 / 麦轮转数，直驱填 1。
    float max_linear_speed_m_s = 0.30f; //!< 距离闭环最大平移速度，单位 m/s。
    float max_angular_speed_rad_s = 1.0f; //!< 距离闭环最大角速度，单位 rad/s。
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
    Enum_Chassis_Wheel wheel = Chassis_Wheel_LeftFront; //!< 该地址对应哪个轮子，记录用。
    Enum_Chassis_Roller_Orientation roller_orientation =
        Chassis_Roller_ForwardSlash;      //!< 滚子方向，只用于人工核对。
    int8_t polarity = 1;                  //!< 电机极性：+1 正向，-1 反向。
    float speed_to_motor = 1.0f;          //!< 由 Init() 按轮径和减速比算出，不用手填。
    Enum_ZDTMotor_Firmware firmware = Enum_ZDTMotor_Firmware::EMM; //!< ZDT 固件协议。
    Enum_ZDTMotor_Checksum checksum = Enum_ZDTMotor_Checksum::FIXED_6B; //!< 校验方式。
    uint32_t pulses_per_revolution = 3200U; //!< EMM 固件每圈脉冲数。
    uint16_t acceleration = 10U; //!< EMM 为 0~255 档位，X 固件为 RPM/s。
};

/** 四轮反馈快照，位置和转速都已应用极性，单位 deg / RPM。 */
struct Struct_Chassis_Wheel_Feedback
{
    float position_deg[CHASSIS_WHEEL_COUNT]{};   //!< 各轮实时位置。
    float speed_rpm[CHASSIS_WHEEL_COUNT]{};      //!< 各轮实时转速。
    uint32_t rx_count[CHASSIS_WHEEL_COUNT]{};    //!< 各轮已通过校验的反馈帧总数。
    uint64_t timestamp_us[CHASSIS_WHEEL_COUNT]{}; //!< 各轮最近一帧反馈的时刻。
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
     * 本接口设置 1 ms 倒计时后立即返回；倒计时结束时同步停止四台电机。
     * 再次调用本接口会以新的速度和时长覆盖尚未结束的定时运行。
     */
    void RunForSeconds(float x, float y, float omega, float duration_s);

    /** 由 TIM_1ms_Task 每 1 ms 调用，维护 RunForSeconds() 的非阻塞倒计时。 */
    void TIM_1ms_RunForSeconds_PeriodElapsedCallback();

    /** 周期调用四台 ZDT 电机的 Process()，维护底层应答和在线状态。 */
    void Process();

    /** 为四台电机缓存停止命令，再发送一次 ZDT 广播同步触发。 */
    void Stop();

    /** 为四台电机缓存失能命令，再发送一次 ZDT 广播同步触发。 */
    void Disable();

    /**
     * @brief 配置四轮周期位置反馈。
     * @param period_ms 反馈周期，0 表示停止该功能码的定时返回。
     * @return 四条配置命令是否全部成功入队。
     */
    bool ConfigureOdometryFeedback(uint16_t period_ms);

    /** 停止四轮周期位置反馈；等价于 ConfigureOdometryFeedback(0)。 */
    bool StopOdometryFeedback();

    /**
     * @brief 复制四轮反馈快照，供 INS 做航位推算。
     * @param feedback 输出快照，位置和转速已应用极性。
     * @return 固定返回 true；本层不判断电机是否在线。
     */
    bool GetWheelFeedback(Struct_Chassis_Wheel_Feedback &feedback) const;

    /**
     * @brief 走到相对当前位姿的目标点，阻塞执行。
     * @param x 机体系方向，`+x` 向右，只有方向有意义，模长不表示距离。
     * @param y 机体系方向，`+y` 向前，只有方向有意义，模长不表示距离。
     * @param omega 相对调用瞬间朝向的目标转角，单位 rad，逆时针为正。
     * @param distance_m 需要行进的距离，单位 m。
     * @return true 已到位并停车；false 参数非法、INS 未就绪或超时。
     *
     * 内部用 osDelay() 等待，只能在 Control_Task 里调用；阻塞期间 InsTask
     * 照常运行，位姿是实时更新的。
     */
    bool RunToRelativePose(float x, float y, float omega, float distance_m);

    /**
     * @brief 修改单轮极性，命令方向和反馈方向同时翻转。
     * @param wheel 目标车轮。
     * @param polarity +1 正向，-1 反向。
     * @return true 修改成功；false 车轮或极性非法。
     * @note 只允许停车时调用，正常运行应优先在 Init() 之前配好极性。
     */
    bool SetWheelPolarity(Enum_Chassis_Wheel wheel, int8_t polarity);

private:
    /** 取消尚未结束的定时运行倒计时。 */
    void CancelRunTimer();

    Struct_Chassis_Config config{}; //!< 底盘尺寸配置副本。
    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{}; //!< 四轮电机配置副本。
    Class_ZDTMotor motors[CHASSIS_WHEEL_COUNT]; //!< 左前、右前、左后、右后电机对象。
    std::atomic<uint32_t> run_remaining_ms{0U}; //!< 定时运行的剩余毫秒数，0 表示未运行。
};

/** Control_Task 与 TIM_1ms_Task 共用的应用层底盘实例。 */
extern Class_Chassis APP_Chassis;

#ifdef __cplusplus
extern "C" {
#endif

/** TIM_1ms_Task 使用的底盘定时运行回调入口。 */
void APP_Chassis_TIM_1ms_RunForSeconds_PeriodElapsedCallback(void);

#ifdef __cplusplus
}
#endif

#endif
