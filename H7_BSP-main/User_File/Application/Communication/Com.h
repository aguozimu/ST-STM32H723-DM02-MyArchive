#ifndef COM_H
#define COM_H

#include <stdbool.h>
#include <stdint.h>

/** 手柄无线帧中的模式字段；只有小陀螺模式会驱动当前底盘。 */
enum class Enum_Remote_Mode : uint8_t
{
    STOP = 0x00U,
    SPEED = 0x01U,
    RA = 0x02U,
    SMALL_GYRO = 0x03U,
};

/** 从 16 字节无线帧解出的原始数据；速度单位换算由控制任务负责。 */
struct Struct_Remote_Control_Frame
{
    uint8_t source = 0U;
    uint8_t sequence = 0U;
    Enum_Remote_Mode mode = Enum_Remote_Mode::STOP;
    int8_t x_velocity_raw = 0;
    int8_t y_velocity_raw = 0;
    uint16_t height_target = 0U;
    uint8_t height_speed = 0U;
    int16_t angle_target = 0;
    int8_t z_angular_velocity_raw = 0;
};

/** Receive counters and the last valid wireless frame, used by VOFA. */
struct Struct_Remote_Control_Debug_State
{
    uint32_t rx_bytes = 0U;
    uint32_t valid_frames = 0U;
    uint32_t invalid_frames = 0U;
    uint32_t last_frame_tick_ms = 0U;
    uint8_t mode = 0U;
    int8_t x_velocity_raw = 0;
    int8_t y_velocity_raw = 0;
    int8_t z_angular_velocity_raw = 0;
};

/** USART10 DMA idle receive callback. */
void Communication_Callback(uint8_t *buffer, uint16_t length);

/** Copy the receive diagnostics snapshot without consuming the control frame. */
void Communication_GetDebugState(Struct_Remote_Control_Debug_State *state);

/** 在任务上下文取走最新有效帧；没有新帧时返回 false。 */
bool Communication_TakeLatestFrame(Struct_Remote_Control_Frame *frame);

#endif // COM_H
