/**
 * @file Control_Task.cpp
 * @brief 初始化底盘并执行一次最简单的坐标位置移动。
 */

 
#include "bsp_uart.h"
#include "user_task.h"
#include "Chassis.h"
#include "INS.h"



#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"
namespace
{
constexpr uint16_t ZDT_ACCELERATION = 10U;
constexpr uint32_t ZDT_PULSES_PER_REVOLUTION = 3200U;
constexpr uint32_t ZDT_BOOT_DELAY_TICKS = 1000U;
constexpr uint32_t ZDT_ENABLE_SETTLE_TICKS = 1000U;
constexpr float CHASSIS_WHEEL_RADIUS_M = 0.05f;
constexpr float CHASSIS_GEAR_RATIO = 1.0f;
volatile bool uart10_rx_ready = false;
volatile uint16_t uart10_rx_length = 0U;
uint8_t uart10_rx_BUFF[UART_BUFFER_SIZE];
uint8_t vision_frame[UART_BUFFER_SIZE];
// static VisionMode vision_mode = V_IDLE;
static uint8_t vision_ref_count = 0U;

void InitChassis()
{
    Struct_Chassis_Config chassis_config{};
    chassis_config.wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    chassis_config.gear_ratio = CHASSIS_GEAR_RATIO;

    Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
    const int8_t motor_polarity[CHASSIS_WHEEL_COUNT] = {
        1,  // 1 号电机：左前轮，正向
        -1, // 2 号电机：右前轮，反向
        1,  // 3 号电机：左后轮，正向
        -1, // 4 号电机：右后轮，反向
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

    /* INS 平移只使用轮式里程计，BMI088 在函数 2 中仅提供 Yaw 融合。 */
    Struct_INS_Config ins_config{};
    ins_config.half_length_m = chassis_config.half_length;
    ins_config.half_width_m = chassis_config.half_width;
    ins_config.wheel_radius_m = chassis_config.wheel_radius_m;
    ins_config.gear_ratio = chassis_config.gear_ratio;
    APP_INS.Init(ins_config);
}
    /*串口10接收回调函数*/
void UART10_RxCallback(uint8_t *buffer, uint16_t length)
{
    if (length > UART_BUFFER_SIZE)
    {
        length = UART_BUFFER_SIZE;
    }
    /*复制内容*/
    memcpy(uart10_rx_BUFF, buffer, length);
    uart10_rx_length = length;
    uart10_rx_ready = true;
}
/* 状态定义 */
enum VisionMode
{
    V_IDLE,       // 空闲
    V_WAIT,       // 已发 YES，等待视觉状态
    V_NOO,        // NOO 移动
    V_RES,        // RES 校准
    V_WAIT_REF    // 已回 OK，等待 3 次 REF
};

static void Vision_Send(const char *data, uint8_t length)
{
    memcpy(USART10_Manage_Object.Tx_Buffer, data, length);
    UART_Transmit_Data(&huart10, USART10_Manage_Object.Tx_Buffer, length);
}

static void Vision_Process(void)
{

    Vision_Send("YES", 3U);
    bool END = false;
    for (;;)
    {
        if (uart10_rx_ready)
        {

            taskENTER_CRITICAL();
            uart10_rx_ready = false;
            const uint16_t length = uart10_rx_length;
            memcpy(vision_frame, uart10_rx_BUFF, length);
            taskEXIT_CRITICAL();

            if (memcmp(vision_frame, "RES", 3U) == 0)
            {
                for (;;)
                {
                    if (uart10_rx_ready)
                    {
                        taskENTER_CRITICAL();
                        uart10_rx_ready = false;
                        const uint16_t length = uart10_rx_length;
                        memcpy(vision_frame, uart10_rx_BUFF, length);
                        taskEXIT_CRITICAL();
                        float vx = 0.0f;
                        float vy = 0.0f;
                        float wz = 0.0f;

                        /* STM32 小端，直接 memcpy 最简单 */
                        memcpy(&vx, &vision_frame[4], 4U);
                        memcpy(&vy, &vision_frame[9], 4U);
                        memcpy(&wz, &vision_frame[14], 4U);
                        APP_Chassis.SetVelocity(vx, vy, wz);
                        if (memcmp(vision_frame, "YES", 3U) == 0)
                        {

                            END = true;
                            break;
                        }
                        else if (memcmp(vision_frame, "REF", 3U) == 0)
                        {

                            break;
                        }
                    
                    }
                 osDelay(1U);
                } 
            }

             if (memcmp(vision_frame, "YES", 3U) == 0)
                        {

                            END = true;
                        
                        }               


        }
        if (END == true)
        {
            Vision_Send("OK", 2U);
            break;
        };

        osDelay(1U);
    }
}

/*结束等待*/
[[noreturn]] void HoldTask()
{
    for (;;)
    {
        osDelay(1000U);
    }
}

} // namespace

extern "C" void Control_Task(void *argument)
{
    (void)argument;
    /*设置当前任务优先级*/
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    /*初始化串口10（视觉通讯）*/
    UART_Init(&huart10, UART10_RxCallback);
    /*等待电机启动*/
    osDelay(ZDT_BOOT_DELAY_TICKS);
    InitChassis();
    APP_Chassis.Enable();
    osDelay(ZDT_ENABLE_SETTLE_TICKS);
    APP_Chassis.Stop();

    Vision_Process();

    APP_Chassis.RunToRelativePosition(-0.95f,
                                      0.0f,
                                      0.01f,
                                      0.0f,
                                      0.05f);

    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   -0.94f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);

    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   0.0f,
    //                                   0.01f,
    //                                   PI / 2.0f,
    //                                   0.01f);

    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   1.25f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);

    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   0.0f,
    //                                   0.01f,
    //                                   -PI / 2.0f,
    //                                   0.01f);
    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   -1.2f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);
    // APP_Chassis.RunToRelativePosition(-1.40f,
    //                                   0.0f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);
    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   0.8f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);

    // APP_Chassis.RunToRelativePosition(0.0f,
    //                                   3.2f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);
    // APP_Chassis.RunToRelativePosition(1.5f,
    //                                   0.0f,
    //                                   0.01f,
    //                                   0.0f,
    //                                   0.05f);

    // APP_Chassis.RunToPosition(0.0f,
    //                           2.1f,
    //                           0.01f,
    //                           0.0f,
    //                           0.05f);
    // // GOHome
    // APP_Chassis.RunToPosition(0.0f,
    //                           0.0f,
    //                           0.01f,
    //                           0.0f,
    //                           0.05f);

    APP_Chassis.Stop();


    HoldTask();
}
