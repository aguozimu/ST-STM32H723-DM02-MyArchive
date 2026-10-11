#ifndef COM_H
#define COM_H

#include <stdbool.h>
#include <stdint.h>

/** 手柄无线帧中的模式字段；具体运动映射由应用层后续配置。 */
enum class Enum_Remote_Mode : uint8_t
{
    STOP = 0x00U,
    SPEED = 0x01U,
    RA = 0x02U,
};

/** 从 16 字节无线帧解出的原始数据，不在接收层做单位或速度映射。 */
struct Struct_Remote_Control_Frame
{
    uint8_t source = 0U;
    uint8_t sequence = 0U;
    Enum_Remote_Mode mode = Enum_Remote_Mode::STOP;
    int8_t left_speed = 0;
    int8_t right_speed = 0;
    uint16_t height_target = 0U;
    uint8_t height_speed = 0U;
    int16_t angle_target = 0;
    int8_t angle_speed = 0;
};

/** USART1 的 DMA 空闲接收回调；只组帧并发布最新有效帧。 */
void Communication_Callback(uint8_t *buffer, uint16_t length);

/** 在任务上下文取走最新有效帧；没有新帧时返回 false。 */
bool Communication_TakeLatestFrame(Struct_Remote_Control_Frame *frame);

#endif // COM_H
