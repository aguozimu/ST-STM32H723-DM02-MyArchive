#include "Com.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstddef>
#include <string.h>

namespace
{
constexpr uint8_t WIRELESS_FRAME_HEADER = 0xFFU;
constexpr uint8_t WIRELESS_FRAME_VERSION = 0x01U;
constexpr uint8_t WIRELESS_FRAME_SIZE = 16U;
constexpr uint8_t WIRELESS_FRAME_TAIL = 0xFEU;

constexpr uint8_t FRAME_SOURCE_OFFSET = 3U;
constexpr uint8_t FRAME_SEQUENCE_OFFSET = 4U;
constexpr uint8_t FRAME_MODE_OFFSET = 5U;
constexpr uint8_t FRAME_LEFT_SPEED_OFFSET = 7U;
constexpr uint8_t FRAME_RIGHT_SPEED_OFFSET = 8U;
constexpr uint8_t FRAME_HEIGHT_TARGET_OFFSET = 9U;
constexpr uint8_t FRAME_HEIGHT_SPEED_OFFSET = 11U;
constexpr uint8_t FRAME_ANGLE_TARGET_OFFSET = 12U;
constexpr uint8_t FRAME_ANGLE_SPEED_OFFSET = 14U;

uint8_t receive_frame[WIRELESS_FRAME_SIZE] = {};
uint8_t receive_length = 0U;
Struct_Remote_Control_Frame latest_frame{};
volatile bool latest_frame_ready = false;

int8_t DecodeInt8(uint8_t raw)
{
    const int16_t value = (raw < 0x80U)
                              ? static_cast<int16_t>(raw)
                              : static_cast<int16_t>(raw) - 0x100;
    return static_cast<int8_t>(value);
}

int16_t DecodeInt16BigEndian(const uint8_t *data)
{
    const uint16_t raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[0]) << 8U) | data[1]);
    const int32_t value = (raw < 0x8000U)
                              ? static_cast<int32_t>(raw)
                              : static_cast<int32_t>(raw) - 0x10000L;
    return static_cast<int16_t>(value);
}

bool DecodeFrame(const uint8_t *data, Struct_Remote_Control_Frame *frame)
{
    if ((data[0] != WIRELESS_FRAME_HEADER) ||
        (data[1] != WIRELESS_FRAME_VERSION) ||
        (data[2] != WIRELESS_FRAME_SIZE) ||
        (data[WIRELESS_FRAME_SIZE - 1U] != WIRELESS_FRAME_TAIL))
    {
        return false;
    }

    frame->source = data[FRAME_SOURCE_OFFSET];
    frame->sequence = data[FRAME_SEQUENCE_OFFSET];
    frame->mode = static_cast<Enum_Remote_Mode>(data[FRAME_MODE_OFFSET]);
    frame->left_speed = DecodeInt8(data[FRAME_LEFT_SPEED_OFFSET]);
    frame->right_speed = DecodeInt8(data[FRAME_RIGHT_SPEED_OFFSET]);
    frame->height_target = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[FRAME_HEIGHT_TARGET_OFFSET]) << 8U) |
        data[FRAME_HEIGHT_TARGET_OFFSET + 1U]);
    frame->height_speed = data[FRAME_HEIGHT_SPEED_OFFSET];
    frame->angle_target = DecodeInt16BigEndian(&data[FRAME_ANGLE_TARGET_OFFSET]);
    frame->angle_speed = DecodeInt8(data[FRAME_ANGLE_SPEED_OFFSET]);
    return true;
}

void PreservePossibleFrameStart(void)
{
    uint8_t next_header = 1U;
    while ((next_header < WIRELESS_FRAME_SIZE) &&
           (receive_frame[next_header] != WIRELESS_FRAME_HEADER))
    {
        ++next_header;
    }

    if (next_header == WIRELESS_FRAME_SIZE)
    {
        receive_length = 0U;
        return;
    }

    receive_length = static_cast<uint8_t>(WIRELESS_FRAME_SIZE - next_header);
    memmove(receive_frame, &receive_frame[next_header], receive_length);
}

void ProcessByte(uint8_t byte)
{
    if (receive_length == 0U)
    {
        if (byte == WIRELESS_FRAME_HEADER)
        {
            receive_frame[0] = byte;
            receive_length = 1U;
        }
        return;
    }

    receive_frame[receive_length] = byte;
    ++receive_length;
    if (receive_length < WIRELESS_FRAME_SIZE)
    {
        return;
    }

    Struct_Remote_Control_Frame decoded_frame{};
    if (DecodeFrame(receive_frame, &decoded_frame))
    {
        latest_frame = decoded_frame;
        latest_frame_ready = true;
        receive_length = 0U;
        return;
    }

    PreservePossibleFrameStart();
}
} // namespace

void Communication_Callback(uint8_t *buffer, uint16_t length)
{
    if (buffer == nullptr)
    {
        return;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        ProcessByte(buffer[i]);
    }
}

bool Communication_TakeLatestFrame(Struct_Remote_Control_Frame *frame)
{
    if (frame == nullptr)
    {
        return false;
    }

    bool has_new_frame = false;
    taskENTER_CRITICAL();
    if (latest_frame_ready)
    {
        *frame = latest_frame;
        latest_frame_ready = false;
        has_new_frame = true;
    }
    taskEXIT_CRITICAL();
    return has_new_frame;
}
