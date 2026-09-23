/**
 * @file InsTask.cpp
 * @brief 100 Hz 位姿更新任务。
 *
 * 每个周期先驱动四台 ZDT 电机的上层处理，再用四轮位置反馈和陀螺仪数据更新
 * x / y / yaw。位姿更新只在这里做，控制闭环只读取结果。
 */

#include "user_task.h"

#include "Chassis.h"
#include "INS.h"

namespace
{
/** 与 INS 融合的标称周期一致的调度周期。 */
constexpr uint32_t INS_PERIOD_MS = 10U;
} // namespace

extern "C" void Ins_Task(void* argument)
{
    (void)argument;

    for (;;)
    {
        APP_Chassis.Process();
        APP_INS.Update();

        osDelay(INS_PERIOD_MS);
    }
}
