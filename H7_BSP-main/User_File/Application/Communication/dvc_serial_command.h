/**
 * @file dvc_serial_command.h
 * @brief UART7 文本运动命令接收与解析。
 */

#ifndef USER_FILE_APPLICATION_COMMUNICATION_DVC_SERIAL_COMMAND_H
#define USER_FILE_APPLICATION_COMMUNICATION_DVC_SERIAL_COMMAND_H

#include "usart.h"

#include <atomic>
#include <stdint.h>

enum class Enum_SerialCommand_Type : uint8_t
{
    NONE = 0U,
    VELOCITY, /*!< V <speed_m_s> <direction_deg> [omega_rad_s] */
    DISTANCE, /*!< D <distance_m> <direction_deg> [omega_rad_s] */
    STOP,     /*!< S */
};

struct Struct_SerialCommand
{
    Enum_SerialCommand_Type type = Enum_SerialCommand_Type::NONE;
    float value = 0.0f;         //!< V: m/s，D: m。
    float direction_deg = 0.0f; //!< 0 向前，+90 向右，-90 向左。
    float omega_rad_s = 0.0f;   //!< 逆时针为正，可省略，默认 0。
};

class Class_SerialCommand
{
public:
    /** @brief 绑定 UART7 并启动逐字节中断接收。 */
    void Init(UART_HandleTypeDef *huart);

    /** @brief 取出一条已解析完成的命令。 */
    bool Read(Struct_SerialCommand &command);

    /** @brief 由 HAL UART RX 完成回调转发。 */
    void RxCpltCallback(UART_HandleTypeDef *huart);

    /** @brief 由 HAL UART 错误回调转发，清理并重启字节接收。 */
    void ErrorCallback(UART_HandleTypeDef *huart);

    uint32_t Get_Rx_Overflow_Count() const;
    uint32_t Get_Rx_Restart_Error_Count() const;
    uint32_t Get_Parse_Error_Count() const;

private:
    static constexpr uint16_t RX_RING_SIZE = 256U;
    static constexpr uint16_t LINE_BUFFER_SIZE = 96U;

    bool Get_Rx_Byte(uint8_t &byte);
    static bool Parse_Command(char *line, Struct_SerialCommand &command);

    UART_HandleTypeDef *UART_Handler = nullptr;
    uint8_t Rx_Byte = 0U;

    uint8_t Rx_Ring[RX_RING_SIZE]{};
    std::atomic<uint16_t> Rx_Head{0U};
    std::atomic<uint16_t> Rx_Tail{0U};
    std::atomic<bool> Rx_Restart_Pending{false};

    char Line_Buffer[LINE_BUFFER_SIZE]{};
    uint16_t Line_Length = 0U;
    bool Discarding_Line = false;

    uint32_t Rx_Overflow_Count = 0U;
    uint32_t Rx_Restart_Error_Count = 0U;
    uint32_t Parse_Error_Count = 0U;
};

extern Class_SerialCommand SerialCommand;

#endif