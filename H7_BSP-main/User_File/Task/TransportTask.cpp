/**
 * @file    TransportTask.cpp
 * @brief   USB CDC init and UART7 VOFA+ telemetry for wireless chassis control.
 */

#include "Com.h"
#include "Chassis.h"
#include "fdcan.h"
#include "usb_device.h"
#include "user_task.h"
#include <string.h>
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr uint8_t VOFA_TELEMETRY_CHANNEL_COUNT = 16U;

float motor_speed_rpm[CHASSIS_WHEEL_COUNT] = {};
float previous_motor_position_deg[CHASSIS_WHEEL_COUNT] = {};
uint32_t previous_speed_tick_ms = 0U;
float can2_bus_off_status = 0.0f;

float NormalizeDeltaDeg(float delta_deg)
{
    delta_deg = std::fmod(delta_deg + 180.0f, 360.0f);
    if (delta_deg < 0.0f)
    {
        delta_deg += 360.0f;
    }
    return delta_deg - 180.0f;
}
Struct_Remote_Control_Debug_State remote_debug{};
float remote_rx_bytes = 0.0f;
float remote_valid_frames = 0.0f;
float remote_invalid_frames = 0.0f;
float remote_mode = 0.0f;
float remote_x_raw = 50.0f;
float remote_y_raw = 50.0f;
float remote_z_raw = 1.0f;
float remote_frame_age_ms = 9999.0f;
float remote_control_active = 0.0f;
float remote_ins_valid = 0.0f;
float remote_motor_online_mask = 0.0f;

volatile bool uart7_rx_ready = false;
volatile uint16_t uart7_rx_length = 0U;
uint8_t uart7_rx_buffer[UART_BUFFER_SIZE];
}

extern "C"
{
extern volatile uint8_t wireless_control_active;
extern volatile uint8_t wireless_ins_valid;
}

void UART7_RxCallback(uint8_t *buffer, uint16_t length)
{
    if (length > UART_BUFFER_SIZE)
    {
        length = UART_BUFFER_SIZE;
    }

    memcpy(uart7_rx_buffer, buffer, length);
    uart7_rx_length = length;
    uart7_rx_ready = true;
}

extern "C" void Transport_Task(void *argument)
{
    (void)argument;
    UART_Init(&huart7, UART7_RxCallback);
    MX_USB_DEVICE_Init();

    EricTool_UART.Set_Data(VOFA_TELEMETRY_CHANNEL_COUNT,
                           (int) &remote_rx_bytes,
                           (int) &remote_valid_frames,
                           (int) &remote_invalid_frames,
                           (int) &remote_mode,
                           (int) &remote_x_raw,
                           (int) &remote_y_raw,
                           (int) &remote_z_raw,
                           (int) &remote_frame_age_ms,
                           (int) &remote_control_active,
                           (int) &remote_ins_valid,
                           (int) &remote_motor_online_mask,
                           (int) &can2_bus_off_status,
                           (int) &motor_speed_rpm[0],
                           (int) &motor_speed_rpm[1],
                           (int) &motor_speed_rpm[2],
                           (int) &motor_speed_rpm[3]);

    for (;;)
    {
        APP_Chassis.Process();

        uint8_t motor_online_mask = 0U;
        const uint32_t sample_tick_ms = HAL_GetTick();
        const uint32_t elapsed_ms = sample_tick_ms - previous_speed_tick_ms;
        for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            const Struct_ZDTMotor_Feedback feedback =
                APP_Chassis.GetMotorFeedback(i);

            if ((previous_speed_tick_ms != 0U) && (elapsed_ms > 0U))
            {
                const float delta_deg = NormalizeDeltaDeg(
                    feedback.position_deg - previous_motor_position_deg[i]);
                motor_speed_rpm[i] =
                    delta_deg * (60.0f / 360.0f) * 1000.0f /
                    static_cast<float>(elapsed_ms);
            }
            previous_motor_position_deg[i] = feedback.position_deg;

            if (feedback.online)
            {
                motor_online_mask |= static_cast<uint8_t>(1U << i);
            }
        }
        previous_speed_tick_ms = sample_tick_ms;

        Communication_GetDebugState(&remote_debug);
        remote_rx_bytes = static_cast<float>(remote_debug.rx_bytes);
        remote_valid_frames = static_cast<float>(remote_debug.valid_frames);
        remote_invalid_frames = static_cast<float>(remote_debug.invalid_frames);
        remote_mode = static_cast<float>(remote_debug.mode);
        remote_x_raw = static_cast<float>(remote_debug.x_velocity_raw);
        remote_y_raw = static_cast<float>(remote_debug.y_velocity_raw);
        remote_z_raw = static_cast<float>(remote_debug.z_angular_velocity_raw);
        remote_frame_age_ms = (remote_debug.valid_frames == 0U)
                                  ? 9999.0f
                                  : static_cast<float>(HAL_GetTick() -
                                                        remote_debug.last_frame_tick_ms);
        remote_control_active = static_cast<float>(wireless_control_active);
        remote_ins_valid = static_cast<float>(wireless_ins_valid);
        remote_motor_online_mask = static_cast<float>(motor_online_mask);

        FDCAN_ProtocolStatusTypeDef can2_protocol_status = {};
        if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &can2_protocol_status) == HAL_OK)
        {
            can2_bus_off_status =
                (can2_protocol_status.BusOff != 0U) ? 1.0f : 0.0f;
        }

        EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback();

        if (uart7_rx_ready)
        {
            bool stop_requested = false;
            taskENTER_CRITICAL();
            uart7_rx_ready = false;
            stop_requested =
                (uart7_rx_length > 0U) && (memcmp(uart7_rx_buffer, "S", 1U) == 0);
            taskEXIT_CRITICAL();

            if (stop_requested)
            {
                APP_Chassis.Stop();
            }
        }

        osDelay(50);
    }
}
