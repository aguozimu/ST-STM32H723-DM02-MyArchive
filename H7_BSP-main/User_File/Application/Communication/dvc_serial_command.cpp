/**
 * @file dvc_serial_command.cpp
 * @brief UART7 文本运动命令接收与解析。
 *
 * 协议：
 *   V <speed_m_s> <direction_deg> [omega_rad_s]\n
 *   D <distance_m> <direction_deg> [omega_rad_s]\n
 *   S\n
 * direction_deg: 0 向前，+90 向右，-90 向左。
 */

#include "dvc_serial_command.h"

#include <cmath>
#include <cstring>

Class_SerialCommand SerialCommand;

namespace
{
bool Is_Space(char c)
{
    return (c == ' ') || (c == '\t');
}

bool Parse_Float(const char *&cursor, float &value)
{
    while (Is_Space(*cursor))
    {
        ++cursor;
    }

    const char *const start = cursor;
    bool negative = false;
    if ((*cursor == '+') || (*cursor == '-'))
    {
        negative = (*cursor == '-');
        ++cursor;
    }

    double number = 0.0;
    bool has_digit = false;
    while ((*cursor >= '0') && (*cursor <= '9'))
    {
        number = number * 10.0 + (double)(*cursor - '0');
        has_digit = true;
        ++cursor;
    }

    if (*cursor == '.')
    {
        ++cursor;
        double scale = 0.1;
        while ((*cursor >= '0') && (*cursor <= '9'))
        {
            number += (double)(*cursor - '0') * scale;
            scale *= 0.1;
            has_digit = true;
            ++cursor;
        }
    }

    if (!has_digit)
    {
        cursor = start;
        return false;
    }

    int exponent = 0;
    bool exponent_negative = false;
    if ((*cursor == 'e') || (*cursor == 'E'))
    {
        ++cursor;
        if ((*cursor == '+') || (*cursor == '-'))
        {
            exponent_negative = (*cursor == '-');
            ++cursor;
        }

        bool exponent_digit = false;
        while ((*cursor >= '0') && (*cursor <= '9') && (exponent < 100))
        {
            exponent = exponent * 10 + (*cursor - '0');
            exponent_digit = true;
            ++cursor;
        }

        if (!exponent_digit)
        {
            cursor = start;
            return false;
        }
    }

    double scale = 1.0;
    for (int i = 0; i < exponent; ++i)
    {
        scale *= 10.0;
    }
    if (exponent_negative)
    {
        number /= scale;
    }
    else
    {
        number *= scale;
    }

    value = (float)(negative ? -number : number);
    return std::isfinite(value);
}

bool Has_Only_Space(const char *cursor)
{
    while (*cursor != '\0')
    {
        if (!Is_Space(*cursor))
        {
            return false;
        }
        ++cursor;
    }
    return true;
}
} // namespace

void Class_SerialCommand::Init(UART_HandleTypeDef *huart)
{
    UART_Handler = huart;
    Rx_Head.store(0U, std::memory_order_relaxed);
    Rx_Tail.store(0U, std::memory_order_relaxed);
    Line_Length = 0U;
    Discarding_Line = false;
    Rx_Overflow_Count = 0U;
    Rx_Restart_Error_Count = 0U;
    Parse_Error_Count = 0U;

    if (UART_Handler != nullptr)
    {
        if (HAL_UART_Receive_IT(UART_Handler, &Rx_Byte, 1U) == HAL_OK)
        {
            Rx_Restart_Pending.store(false, std::memory_order_release);
        }
        else
        {
            Rx_Restart_Pending.store(true, std::memory_order_release);
            ++Rx_Restart_Error_Count;
        }
    }
}

void Class_SerialCommand::RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((UART_Handler == nullptr) || (huart != UART_Handler))
    {
        return;
    }

    const uint16_t head = Rx_Head.load(std::memory_order_relaxed);
    const uint16_t next_head = (uint16_t)((head + 1U) % RX_RING_SIZE);
    if (next_head != Rx_Tail.load(std::memory_order_acquire))
    {
        Rx_Ring[head] = Rx_Byte;
        Rx_Head.store(next_head, std::memory_order_release);
    }
    else
    {
        ++Rx_Overflow_Count;
    }

    if (HAL_UART_Receive_IT(UART_Handler, &Rx_Byte, 1U) == HAL_OK)
    {
        Rx_Restart_Pending.store(false, std::memory_order_release);
    }
    else
    {
        Rx_Restart_Pending.store(true, std::memory_order_release);
        ++Rx_Restart_Error_Count;
    }
}

void Class_SerialCommand::ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((UART_Handler == nullptr) || (huart != UART_Handler))
    {
        return;
    }

    ++Rx_Restart_Error_Count;
    if (HAL_UART_Receive_IT(UART_Handler, &Rx_Byte, 1U) == HAL_OK)
    {
        Rx_Restart_Pending.store(false, std::memory_order_release);
    }
    else
    {
        Rx_Restart_Pending.store(true, std::memory_order_release);
    }
}

bool Class_SerialCommand::Get_Rx_Byte(uint8_t &byte)
{
    const uint16_t tail = Rx_Tail.load(std::memory_order_relaxed);
    if (tail == Rx_Head.load(std::memory_order_acquire))
    {
        return false;
    }

    byte = Rx_Ring[tail];
    Rx_Tail.store((uint16_t)((tail + 1U) % RX_RING_SIZE), std::memory_order_release);
    return true;
}

bool Class_SerialCommand::Parse_Command(char *line, Struct_SerialCommand &command)
{
    char *cursor = line;
    while (Is_Space(*cursor))
    {
        ++cursor;
    }

    if (*cursor == '\0')
    {
        return false;
    }

    char command_char = *cursor;
    if ((command_char >= 'a') && (command_char <= 'z'))
    {
        command_char = (char)(command_char - 'a' + 'A');
    }
    ++cursor;

    if (command_char == 'S')
    {
        if (!Has_Only_Space(cursor))
        {
            return false;
        }
        command = {};
        command.type = Enum_SerialCommand_Type::STOP;
        return true;
    }

    if ((command_char != 'V') && (command_char != 'D'))
    {
        return false;
    }

    const char *number_cursor = cursor;
    float value = 0.0f;
    float direction_deg = 0.0f;
    float omega_rad_s = 0.0f;

    if (!Parse_Float(number_cursor, value) ||
        !Parse_Float(number_cursor, direction_deg) || (value < 0.0f))
    {
        return false;
    }

    while (Is_Space(*number_cursor))
    {
        ++number_cursor;
    }
    if (*number_cursor != '\0')
    {
        if (!Parse_Float(number_cursor, omega_rad_s) ||
            !Has_Only_Space(number_cursor))
        {
            return false;
        }
    }

    command = {};
    command.value = value;
    command.direction_deg = direction_deg;
    command.omega_rad_s = omega_rad_s;
    command.type = command_char == 'V' ? Enum_SerialCommand_Type::VELOCITY
                                       : Enum_SerialCommand_Type::DISTANCE;
    return true;
}

bool Class_SerialCommand::Read(Struct_SerialCommand &command)
{
    if (Rx_Restart_Pending.load(std::memory_order_acquire))
    {
        if (HAL_UART_Receive_IT(UART_Handler, &Rx_Byte, 1U) == HAL_OK)
        {
            Rx_Restart_Pending.store(false, std::memory_order_release);
        }
    }

    uint8_t byte = 0U;
    while (Get_Rx_Byte(byte))
    {
        if (Discarding_Line)
        {
            if ((byte == '\r') || (byte == '\n'))
            {
                Discarding_Line = false;
            }
            continue;
        }

        if ((byte == '\r') || (byte == '\n'))
        {
            if (Line_Length == 0U)
            {
                continue;
            }

            Line_Buffer[Line_Length] = '\0';
            Struct_SerialCommand parsed{};
            if (Parse_Command(Line_Buffer, parsed))
            {
                command = parsed;
                Line_Length = 0U;
                return true;
            }

            ++Parse_Error_Count;
            Line_Length = 0U;
            continue;
        }

        if (Line_Length < (LINE_BUFFER_SIZE - 1U))
        {
            Line_Buffer[Line_Length] = (char)byte;
            ++Line_Length;
        }
        else
        {
            // 超长行整体丢弃，等待下一个换行符重新同步。
            Line_Length = 0U;
            Discarding_Line = true;
            ++Parse_Error_Count;
        }
    }

    return false;
}

uint32_t Class_SerialCommand::Get_Rx_Overflow_Count() const
{
    return Rx_Overflow_Count;
}

uint32_t Class_SerialCommand::Get_Rx_Restart_Error_Count() const
{
    return Rx_Restart_Error_Count;
}

uint32_t Class_SerialCommand::Get_Parse_Error_Count() const
{
    return Parse_Error_Count;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    SerialCommand.RxCpltCallback(huart);
}