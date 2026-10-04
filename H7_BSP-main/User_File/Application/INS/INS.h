/**
 * @file INS.h
 * @brief 陀螺仪 Yaw 与四轮里程计的最简互补融合。
 *
 * 本层只做一件事：按固定周期把四轮位置增量、BMI088 z 轴角速度和计时间隔
 * 融合成世界系位姿 x / y / yaw。X/Y 只使用轮式正运动学，BMI088 只参与
 * Yaw 互补融合，不进行加速度二次积分和打滑检测。
 */

#ifndef USER_FILE_APPLICATION_INS_INS_H
#define USER_FILE_APPLICATION_INS_INS_H

#include "Chassis.h"
#include "cmsis_os2.h"

#include <atomic>
#include <stdint.h>

/** INS 更新周期，单位 ms。 */
static constexpr uint16_t INS_UPDATE_PERIOD_MS = 10U;
/** 航向互补滤波中陀螺仪的默认权重。 */
static constexpr float INS_DEFAULT_GYRO_WEIGHT = 0.998f;
 //0.98f;
/** INS 机械参数。非直驱时 gear_ratio 表示电机转数 / 车轮转数。 */
struct Struct_INS_Config
{
    float half_length_m = 0.075f;  //!< 底盘中心到前/后轮轴的距离，单位 m。
    float half_width_m = 0.205f;   //!< 底盘中心到左/右轮接触中心的距离，单位 m。
    float wheel_radius_m = 0.0375f; //!< 麦轮有效滚动半径，单位 m。
    float gear_ratio = 1.0f;      //!< 电机转数 / 车轮转数，直驱为 1。
    float gyro_weight = INS_DEFAULT_GYRO_WEIGHT; //!< 互补滤波的陀螺仪权重，范围 0~1。
};

/** 二维位姿快照；复位瞬间的车体系就是世界系。 */
struct Struct_INS_Pose
{
    float x_m = 0.0f;     //!< 世界系 x 坐标，单位 m，+x 为复位时的车体右侧。
    float y_m = 0.0f;     //!< 世界系 y 坐标，单位 m，+y 为复位时的车头方向。
    float yaw_rad = 0.0f; //!< 航向角，单位 rad，逆时针为正。
    bool valid = false;   //!< 输入基准已建立且该位姿至少更新过一次。
};

/** 三个处理步骤所维护的位姿快照。 */
struct Struct_INS_State
{
    Struct_INS_Pose wheel;         //!< 函数 1：四轮编码器正运动学位姿。
    Struct_INS_Pose gyro;          //!< 函数 2：陀螺仪航向位姿。
    Struct_INS_Pose complementary; //!< 函数 3：互补滤波后位姿。
};

/** 世界坐标系校准记录：保存校准开始时的世界系快照。 */
struct Struct_INS_Calibration
{
    bool active = false;            //!< 是否处于一次校准流程中。
    Struct_INS_State start_state{}; //!< 校准开始时记录的三组世界系位姿。
};

/** 按 Chassis 应用层风格实现的最简底盘 INS。 */
class Class_INS
{
public:
    /**
     * @brief 保存参数、配置四轮 10 ms 位置反馈并复位位姿。
     * @param config 轮径、减速比和互补滤波权重。
     *
     * 应在 Class_Chassis::Init() 之后调用。该函数会再次调用
     * APP_Chassis.ConfigurePeriodicFeedback(INS_UPDATE_PERIOD_MS)。
     */
    void Init(const Struct_INS_Config &config);

    /** 清零三个位姿和所有差分基准，不修改电机内部位置。 */
    void Reset();

    /** 函数 1：处理四轮编码器反馈并执行麦轮正运动学，周期 10 ms。 */
    void UpdateWheelPosition();

    /** 函数 2：积分 BMI088 陀螺仪 Yaw，周期 10 ms。 */
    void UpdateGyroPosition();

    /** 函数 3：对轮式和陀螺仪航向增量做互补融合，周期 10 ms。 */
    void UpdateComplementaryPosition();

    /** 分别读取三个处理步骤的位姿快照。 */
    Struct_INS_Pose GetWheelPosition() const;
    Struct_INS_Pose GetGyroPosition() const;
    Struct_INS_Pose GetComplementaryPosition() const;

    /** 一次性读取三个位姿快照。 */
    Struct_INS_State GetState() const;

    /**
     * @brief 世界坐标系校准开始：记录当前世界系相关的全部数据。
     * @return 记录成功返回 true；世界系位姿尚未有效时返回 false。
     *
     * 由外部任务在开始校准移动（SetVelocity）之前调用一次。函数会把函数 1、
     * 2、3 的三组世界系位姿完整保存下来，作为校准结束时计算运行误差的基准。
     */
    bool StartCalibration();

    /**
     * @brief 世界坐标系校准结束：用本次运行的误差修正世界系原点。
     * @return 修正成功返回 true；未调用 StartCalibration() 时返回 false。
     *
     * 读取当前世界系位姿与校准开始时记录位姿的差值，即本次运行累计产生的
     * 误差，再从三组世界系位姿中扣除，等价于把世界系原点重新对齐到校准
     * 起点。三个位姿同处一个世界系，统一使用函数 3 互补滤波位姿的误差：
     *
     *     error = current - recorded
     *     pose  = pose - error
     */
    bool EndCalibration();

private:
    /** 加锁/解锁；Init() 之前锁对象未创建时为空操作。 */
    void Lock() const;
    void Unlock() const;

    /** 作用域互斥锁：构造时加锁、析构时解锁，避免提前 return 漏解锁。 */
    class Class_Lock_Guard
    {
    public:
        explicit Class_Lock_Guard(const Class_INS &owner) : owner_(owner)
        {
            owner_.Lock();
        }

        ~Class_Lock_Guard()
        {
            owner_.Unlock();
        }

        Class_Lock_Guard(const Class_Lock_Guard &) = delete;
        Class_Lock_Guard &operator=(const Class_Lock_Guard &) = delete;

    private:
        const Class_INS &owner_;
    };

    /** 在已加锁的前提下清零三个位姿和所有差分基准。 */
    void ResetState();

    Struct_INS_Config config{}; //!< 已完成合法性修正的机械参数。

    Struct_INS_Pose wheel_position{};         //!< 函数 1 输出。
    Struct_INS_Pose gyro_position{};          //!< 函数 2 输出。
    Struct_INS_Pose complementary_position{}; //!< 函数 3 输出。

    float last_wheel_position_deg[CHASSIS_WHEEL_COUNT]{}; //!< 各轮上一位置，单位 deg。
    uint32_t last_wheel_rx_count[CHASSIS_WHEEL_COUNT]{};  //!< 各轮上一反馈帧计数。
    bool wheel_reference_ready[CHASSIS_WHEEL_COUNT]{};    //!< 各轮位置基准是否已建立。
    bool wheel_pending_ready[CHASSIS_WHEEL_COUNT]{};      //!< 各轮是否已累计新位移。
    float wheel_pending_m[CHASSIS_WHEEL_COUNT]{};         //!< 各轮待合成的位移，单位 m。

    uint32_t wheel_delta_sequence = 0U; //!< 完整四轮位移增量的发布序号。
    float wheel_delta_x_m = 0.0f;       //!< 最近一组机体系 x 位移增量。
    float wheel_delta_y_m = 0.0f;       //!< 最近一组机体系 y 位移增量。
    float wheel_delta_yaw_rad = 0.0f;   //!< 最近一组轮式航向增量。

    uint64_t last_gyro_timestamp_us = 0U; //!< 上一次陀螺仪积分时间戳。
    uint32_t gyro_delta_sequence = 0U;    //!< 陀螺仪航向增量发布序号。
    float gyro_delta_yaw_rad = 0.0f;      //!< 最近一次陀螺仪航向增量。

    uint32_t gyro_used_wheel_sequence = 0U;          //!< 函数 2 已使用的轮式平移序号。
    uint32_t complementary_used_wheel_sequence = 0U; //!< 函数 3 已使用的轮式序号。
    uint32_t complementary_used_gyro_sequence = 0U;  //!< 函数 3 已使用的陀螺仪序号。

    Struct_INS_Calibration calibration{}; //!< 世界系校准记录。
    osMutexId_t pose_mutex = nullptr;     //!< 保护三组位姿和校准数据的互斥锁。

    std::atomic<bool> initialized{false}; //!< Init() 完成后才允许周期函数更新。
};

/** 各任务共用的 INS 应用层实例。 */
extern Class_INS APP_INS;

#ifdef __cplusplus
extern "C" {
#endif

/** TIM1msTask 的 10 ms 回调：函数 1。 */
void INS_TIM_10ms_UpdateWheelPosition_PeriodElapsedCallback(void);
/** TIM1msTask 的 10 ms 回调：函数 2。 */
void INS_TIM_10ms_UpdateGyroPosition_PeriodElapsedCallback(void);
/** TIM1msTask 的 10 ms 回调：函数 3。 */
void INS_TIM_10ms_UpdateComplementaryPosition_PeriodElapsedCallback(void);

#ifdef __cplusplus
}
#endif

#endif
