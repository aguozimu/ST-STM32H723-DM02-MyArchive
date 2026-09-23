/**
 * @file INS.h
 * @brief 陀螺仪与四轮里程计的最简航向/位置融合。
 *
 * 本层只做一件事：按固定周期把四轮位置增量、陀螺仪 z 轴角速度和计时间隔
 * 融合成机体系位姿 x / y / yaw。不判有效、不判超时、不判打滑。
 */

#ifndef USER_FILE_APPLICATION_INS_INS_H
#define USER_FILE_APPLICATION_INS_INS_H

#include "Chassis.h"

#include <stdint.h>

/** 惯性导航输出的位姿快照；复位瞬间机体系就是世界系。 */
struct Struct_INS_State
{
    float x_m = 0.0f;     //!< 世界系 x 坐标，单位 m，`+x` 为复位时的车体右侧。
    float y_m = 0.0f;     //!< 世界系 y 坐标，单位 m，`+y` 为复位时的车头方向。
    float yaw_rad = 0.0f; //!< 世界系航向角，单位 rad，逆时针为正。
    bool valid = false;   //!< 是否已经建立过基准并输出过位姿。
};

/** 视觉绝对定位观测；第一版只保存最新一帧，不做融合。 */
struct Struct_Vision_Pose_Observation
{
    float x_m = 0.0f;        //!< 视觉给出的世界系 x 坐标，单位 m。
    float y_m = 0.0f;        //!< 视觉给出的世界系 y 坐标，单位 m。
    float yaw_rad = 0.0f;    //!< 视觉给出的世界系航向角，单位 rad。
    uint64_t timestamp_us = 0U; //!< 采集时刻，不是接收时刻。
    bool valid = false;      //!< 本帧是否可用。
};

/** 最简互补融合的惯性导航。 */
class Class_INS
{
public:
    /**
     * @brief 用底盘配置里的轮径、减速比和半长/半宽初始化。
     * @param config 与 `Class_Chassis::Init()` 使用同一份配置。
     */
    void Init(const Struct_Chassis_Config &config);

    /** 位姿清零，不动电机内部位置。 */
    void ResetPose();

    /**
     * @brief 融合一帧四轮位置反馈和陀螺仪数据，更新位姿。
     * @return true 本次已更新位姿；false 时间戳异常，本周期未积分。
     * @note 由 InsTask 以 10 ms 周期调用，不要放在控制闭环里。
     */
    bool Update();

    /** 取最近一次融合得到的位姿快照。 */
    Struct_INS_State GetState() const;

    /**
     * @brief 保存最新一帧视觉绝对定位观测。
     * @note 第一版只缓存，不参与融合，接入时再补坐标系变换和时间同步。
     */
    void UpdateVisionPose(const Struct_Vision_Pose_Observation &observation);

    /** 取最近一帧视觉绝对定位观测。 */
    Struct_Vision_Pose_Observation GetVisionPose() const;

private:
    float half_length = 0.15f;  //!< 底盘中心到前/后轮轴距离，单位 m。
    float half_width = 0.15f;   //!< 底盘中心到左/右轮接触中心距离，单位 m。
    float wheel_radius_m = 0.05f; //!< 麦轮有效滚动半径，单位 m。
    float gear_ratio = 1.0f;    //!< 电机转数 / 麦轮转数，直驱为 1。

    Struct_INS_State state{};   //!< 当前位姿。
    Struct_Vision_Pose_Observation vision_pose{}; //!< 最近一帧视觉观测，暂不融合。
    float last_position_deg[CHASSIS_WHEEL_COUNT] = {0.0f}; //!< 上一周期各轮位置，单位 deg。
    bool wheel_reference_ready[CHASSIS_WHEEL_COUNT] = {false}; //!< 各轮是否已建立位置基准。
    uint64_t last_timestamp_us = 0U; //!< 上一周期时间戳，单位 us。
    bool first_sample = true;        //!< 是否还没建立第一帧基准。
};

/** 与 APP_Chassis 一样，全局共享同一个 INS 实例。 */
extern Class_INS APP_INS;

#endif
