/**
 * @file zdtmotor.cpp
 * @brief ZDT X42S CAN 命令封装、分包、反馈解析与运行状态管理。
 * @author zzm
 */

#include "zdtmotor.h"

#include "sys_timestamp.h"

#include <cmath>
#include <cstring>

static constexpr uint8_t ZDT_MAX_COMMAND_BODY = 16U;
static constexpr uint8_t ZDT_MAX_COMMAND_FRAMES = 3U;
static constexpr uint64_t ZDT_RESPONSE_TIMEOUT_US = 50000U;
static constexpr uint64_t ZDT_OFFLINE_TIMEOUT_US = 500000U;

static uint16_t ZDT_ReadBE16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static uint32_t ZDT_ReadBE32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           data[3];
}

static void ZDT_WriteBE16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

static void ZDT_WriteBE32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

static bool ZDT_IsFiniteNonNegative(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}

static bool ZDT_IsActionFunction(uint8_t function)
{
    switch (function)
    {
    case 0x08U:
    case 0x0AU:
    case 0x0EU:
    case 0x11U:
    case 0x9AU:
    case 0x9CU:
    case 0xF3U:
    case 0xF5U:
    case 0xF6U:
    case 0xFDU:
    case 0xFEU:
    case 0xFFU:
        return true;

    default:
        return false;
    }
}

bool Class_ZDTMotor::Init(FDCAN_HandleTypeDef *motor_hfdcan,
                          uint8_t motor_address,
                          Enum_ZDTMotor_Firmware motor_firmware,
                          Enum_ZDTMotor_Checksum motor_checksum,
                          bool motor_reverse,
                          uint32_t motor_pulses_per_revolution)
{
    if (motor_hfdcan == nullptr || motor_address == 0U ||
        motor_pulses_per_revolution == 0U ||
        (motor_firmware != Enum_ZDTMotor_Firmware::X &&
         motor_firmware != Enum_ZDTMotor_Firmware::EMM) ||
        (motor_checksum != Enum_ZDTMotor_Checksum::FIXED_6B &&
         motor_checksum != Enum_ZDTMotor_Checksum::XOR &&
         motor_checksum != Enum_ZDTMotor_Checksum::CRC8))
    {
        return false;
    }

    hfdcan = motor_hfdcan;
    address = motor_address;
    firmware = motor_firmware;
    checksum = motor_checksum;
    reverse = motor_reverse;
    pulses_per_revolution = motor_pulses_per_revolution;
    motion_state = Enum_ZDTMotor_Motion_State::IDLE;
    tracked_function = 0U;
    command_timestamp_us = 0U;
    feedback = Struct_ZDTMotor_Feedback{};

    return BSP_CAN_RegisterCallbackEx((uint32_t)address << 8U,
                                      CAN_ID_EXTENDED,
                                      hfdcan,
                                      FeedbackCallback,
                                      this);
}

uint8_t Class_ZDTMotor::UpdateCRC8(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        crc = (crc & 0x01U) != 0U
                  ? (uint8_t)((crc >> 1U) ^ 0x8CU)
                  : (uint8_t)(crc >> 1U);
    }
    return crc;
}

uint8_t Class_ZDTMotor::CalculateCommandChecksum(uint8_t target_address,
                                                 uint8_t function,
                                                 const uint8_t *body,
                                                 uint8_t body_length) const
{
    if (checksum == Enum_ZDTMotor_Checksum::FIXED_6B)
    {
        return 0x6BU;
    }

    if (checksum == Enum_ZDTMotor_Checksum::XOR)
    {
        uint8_t result = (uint8_t)(target_address ^ function);
        for (uint8_t index = 0U; index < body_length; index++)
        {
            result ^= body[index];
        }
        return result;
    }

    uint8_t result = target_address;
    result = UpdateCRC8(result, function);
    for (uint8_t index = 0U; index < body_length; index++)
    {
        result = UpdateCRC8(result, body[index]);
    }
    return result;
}

bool Class_ZDTMotor::CheckResponseChecksum(const uint8_t *data, uint8_t length) const
{
    if (data == nullptr || length < 2U)
    {
        return false;
    }

    if (checksum == Enum_ZDTMotor_Checksum::FIXED_6B)
    {
        return data[length - 1U] == 0x6BU;
    }

    if (checksum == Enum_ZDTMotor_Checksum::XOR)
    {
        uint8_t result = address;
        for (uint8_t index = 0U; index < length - 1U; index++)
        {
            result ^= data[index];
        }
        return result == data[length - 1U];
    }

    uint8_t result = address;
    for (uint8_t index = 0U; index < length - 1U; index++)
    {
        result = UpdateCRC8(result, data[index]);
    }
    return result == data[length - 1U];
}

bool Class_ZDTMotor::SubmitCommand(uint8_t target_address,
                                   uint8_t function,
                                   const uint8_t *body,
                                   uint8_t body_length,
                                   bool track_motion)
{
    Struct_CAN_Tx_Msg messages[ZDT_MAX_COMMAND_FRAMES]{};
    uint8_t tail[ZDT_MAX_COMMAND_BODY + 1U]{};

    if (hfdcan == nullptr || body_length > ZDT_MAX_COMMAND_BODY ||
        (body_length > 0U && body == nullptr))
    {
        return false;
    }

    if (body_length > 0U)
    {
        std::memcpy(tail, body, body_length);
    }
    tail[body_length] = CalculateCommandChecksum(target_address,
                                                 function,
                                                 body,
                                                 body_length);

    const uint8_t tail_length = (uint8_t)(body_length + 1U);
    const uint8_t frame_count = (uint8_t)((tail_length + 6U) / 7U);
    if (frame_count == 0U || frame_count > ZDT_MAX_COMMAND_FRAMES)
    {
        return false;
    }

    uint8_t tail_offset = 0U;
    for (uint8_t packet = 0U; packet < frame_count; packet++)
    {
        const uint8_t remaining = (uint8_t)(tail_length - tail_offset);
        const uint8_t chunk_length = remaining > 7U ? 7U : remaining;
        messages[packet].hfdcan = hfdcan;
        messages[packet].id = ((uint32_t)target_address << 8U) | packet;
        messages[packet].id_type = CAN_ID_EXTENDED;
        messages[packet].data[0] = function;
        std::memcpy(&messages[packet].data[1], &tail[tail_offset], chunk_length);
        messages[packet].len = (uint8_t)(chunk_length + 1U);
        tail_offset = (uint8_t)(tail_offset + chunk_length);
    }

    if (!CAN_Tx_SubmitBatch(messages, frame_count))
    {
        return false;
    }

    if (track_motion)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        tracked_function = function;
        command_timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
        motion_state = Enum_ZDTMotor_Motion_State::WAIT_ACCEPT;
        __DMB();
        if (primask == 0U)
        {
            __enable_irq();
        }
    }

    return true;
}

bool Class_ZDTMotor::Enable(bool synchronous)
{
    const uint8_t body[] = {0xABU, 0x01U, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    return SubmitCommand(address, 0xF3U, body, sizeof(body), false);
}

bool Class_ZDTMotor::Disable(bool synchronous)
{
    const uint8_t body[] = {0xABU, 0x00U, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    return SubmitCommand(address, 0xF3U, body, sizeof(body), false);
}

bool Class_ZDTMotor::Stop(bool synchronous)
{
    const uint8_t body[] = {0x98U, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    if (!SubmitCommand(address, 0xFEU, body, sizeof(body), false))
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motion_state = Enum_ZDTMotor_Motion_State::IDLE;
    tracked_function = 0U;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

bool Class_ZDTMotor::ClearPosition()
{
    const uint8_t body[] = {0x6DU};
    return SubmitCommand(address, 0x0AU, body, sizeof(body), false);
}

bool Class_ZDTMotor::ClearProtection()
{
    const uint8_t body[] = {0x52U};
    return SubmitCommand(address, 0x0EU, body, sizeof(body), false);
}

bool Class_ZDTMotor::Restart()
{
    const uint8_t body[] = {0x97U};
    return SubmitCommand(address, 0x08U, body, sizeof(body), false);
}

bool Class_ZDTMotor::TriggerHoming(Enum_ZDTMotor_Homing_Mode mode,
                                   bool synchronous)
{
    if ((uint8_t)mode > (uint8_t)Enum_ZDTMotor_Homing_Mode::POWER_OFF_POSITION)
    {
        return false;
    }

    const uint8_t body[] = {(uint8_t)mode, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    return SubmitCommand(address, 0x9AU, body, sizeof(body), true);
}

bool Class_ZDTMotor::AbortHoming()
{
    const uint8_t body[] = {0x48U};
    return SubmitCommand(address, 0x9CU, body, sizeof(body), false);
}

bool Class_ZDTMotor::TriggerSynchronousMotion()
{
    const uint8_t body[] = {0x66U};
    return SubmitCommand(0U, 0xFFU, body, sizeof(body), false);
}

bool Class_ZDTMotor::SetSpeed(float speed_rpm,
                              uint16_t acceleration,
                              bool synchronous)
{
    if (!std::isfinite(speed_rpm))
    {
        return false;
    }

    float directed_speed = reverse ? -speed_rpm : speed_rpm;
    const uint8_t direction = directed_speed < 0.0f ? 0x01U : 0x00U;
    const float magnitude = std::fabs(directed_speed);
    uint8_t body[6]{};
    body[0] = direction;

    if (firmware == Enum_ZDTMotor_Firmware::X)
    {
        if (magnitude > 3000.0f)
        {
            return false;
        }
        ZDT_WriteBE16(&body[1], acceleration);
        ZDT_WriteBE16(&body[3], (uint16_t)(magnitude * 10.0f + 0.5f));
        body[5] = synchronous ? 0x01U : 0x00U;
        return SubmitCommand(address, 0xF6U, body, 6U, true);
    }

    if (magnitude > 3000.0f || acceleration > 255U)
    {
        return false;
    }
    ZDT_WriteBE16(&body[1], (uint16_t)(magnitude + 0.5f));
    body[3] = (uint8_t)acceleration;
    body[4] = synchronous ? 0x01U : 0x00U;
    return SubmitCommand(address, 0xF6U, body, 5U, true);
}

bool Class_ZDTMotor::SetPosition(float position_deg,
                                 float speed_rpm,
                                 uint16_t acceleration,
                                 uint16_t deceleration,
                                 Enum_ZDTMotor_Position_Mode mode,
                                 bool synchronous)
{
    if (!std::isfinite(position_deg) || !ZDT_IsFiniteNonNegative(speed_rpm) ||
        speed_rpm > 3000.0f ||
        (uint8_t)mode > (uint8_t)Enum_ZDTMotor_Position_Mode::RELATIVE_TO_CURRENT)
    {
        return false;
    }

    const float directed_position = reverse ? -position_deg : position_deg;
    const uint8_t direction = directed_position < 0.0f ? 0x01U : 0x00U;
    const float magnitude = std::fabs(directed_position);

    if (firmware == Enum_ZDTMotor_Firmware::X)
    {
        const double encoded_position = (double)magnitude * 10.0;
        if (encoded_position > 4294967295.0)
        {
            return false;
        }

        uint8_t body[13]{};
        body[0] = direction;
        ZDT_WriteBE16(&body[1], acceleration);
        ZDT_WriteBE16(&body[3], deceleration);
        ZDT_WriteBE16(&body[5], (uint16_t)(speed_rpm * 10.0f + 0.5f));
        ZDT_WriteBE32(&body[7], (uint32_t)(encoded_position + 0.5));
        body[11] = (uint8_t)mode;
        body[12] = synchronous ? 0x01U : 0x00U;
        return SubmitCommand(address, 0xFDU, body, sizeof(body), true);
    }

    if (acceleration > 255U)
    {
        return false;
    }

    const double pulses = (double)magnitude * (double)pulses_per_revolution / 360.0;
    if (pulses > 4294967295.0)
    {
        return false;
    }

    uint8_t body[10]{};
    body[0] = direction;
    ZDT_WriteBE16(&body[1], (uint16_t)(speed_rpm + 0.5f));
    body[3] = (uint8_t)acceleration;
    ZDT_WriteBE32(&body[4], (uint32_t)(pulses + 0.5));
    body[8] = (uint8_t)mode;
    body[9] = synchronous ? 0x01U : 0x00U;
    return SubmitCommand(address, 0xFDU, body, sizeof(body), true);
}

bool Class_ZDTMotor::SetTorque(float current_a,
                               uint16_t current_slope_ma_s,
                               bool synchronous)
{
    if (firmware != Enum_ZDTMotor_Firmware::X || !std::isfinite(current_a))
    {
        return false;
    }

    const float directed_current = reverse ? -current_a : current_a;
    const float magnitude = std::fabs(directed_current);
    if (magnitude > 5.0f)
    {
        return false;
    }

    uint8_t body[6]{};
    body[0] = directed_current < 0.0f ? 0x01U : 0x00U;
    ZDT_WriteBE16(&body[1], current_slope_ma_s);
    ZDT_WriteBE16(&body[3], (uint16_t)(magnitude * 1000.0f + 0.5f));
    body[5] = synchronous ? 0x01U : 0x00U;
    return SubmitCommand(address, 0xF5U, body, sizeof(body), true);
}

bool Class_ZDTMotor::ReadVersion()
{
    return SubmitCommand(address, 0x1FU, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadOptionStatus()
{
    return SubmitCommand(address, 0x1AU, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadBusVoltage()
{
    return SubmitCommand(address, 0x24U, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadPhaseCurrent()
{
    return SubmitCommand(address, 0x27U, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadSpeed()
{
    return SubmitCommand(address, 0x35U, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadPosition()
{
    return SubmitCommand(address, 0x36U, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadPositionError()
{
    return SubmitCommand(address, 0x37U, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadTemperature()
{
    return SubmitCommand(address, 0x39U, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadMotorStatus()
{
    return SubmitCommand(address, 0x3AU, nullptr, 0U, false);
}

bool Class_ZDTMotor::ReadHomingStatus()
{
    return SubmitCommand(address, 0x3BU, nullptr, 0U, false);
}

bool Class_ZDTMotor::ConfigurePeriodicFeedback(uint8_t function,
                                               uint16_t period_ms)
{
    const uint8_t body[] = {
        0x18U,
        function,
        (uint8_t)(period_ms >> 8U),
        (uint8_t)period_ms,
    };
    return SubmitCommand(address, 0x11U, body, sizeof(body), false);
}

float Class_ZDTMotor::ApplyDirection(float value) const
{
    return reverse ? -value : value;
}

void Class_ZDTMotor::ParseFeedback(const uint8_t *data, uint8_t length)
{
    const uint8_t function = data[0];
    feedback.last_function = function;

    if (length == 3U &&
        (ZDT_IsActionFunction(function) || data[1] == 0xE2U || data[1] == 0xEEU))
    {
        const uint8_t response = data[1];
        if (response == 0x02U || response == 0x12U || response == 0x22U ||
            response == 0x9FU || response == 0xE2U || response == 0xEEU)
        {
            feedback.last_response = response;
            if (function == tracked_function)
            {
                if (response == 0x02U)
                {
                    motion_state = Enum_ZDTMotor_Motion_State::RUNNING;
                }
                else if (response == 0x9FU)
                {
                    motion_state = Enum_ZDTMotor_Motion_State::DONE;
                }
                else
                {
                    motion_state = Enum_ZDTMotor_Motion_State::ERROR;
                }
            }
        }
    }

    switch (function)
    {
    case 0x1FU:
        if (length == 6U)
        {
            feedback.firmware_version = ZDT_ReadBE16(&data[1]);
            feedback.hardware_series = (data[3] >> 4U) & 0x0FU;
            feedback.hardware_type = data[3] & 0x0FU;
            feedback.hardware_version = data[4];
        }
        break;

    case 0x1AU:
        if (length == 3U)
        {
            feedback.option_status = data[1];
            feedback.reported_firmware = (data[1] & 0x02U) != 0U
                                             ? Enum_ZDTMotor_Firmware::EMM
                                             : Enum_ZDTMotor_Firmware::X;
        }
        break;

    case 0x24U:
        if (length == 4U)
        {
            feedback.bus_voltage_v = (float)ZDT_ReadBE16(&data[1]) / 1000.0f;
        }
        break;

    case 0x27U:
        if (length == 4U)
        {
            feedback.phase_current_a = (float)ZDT_ReadBE16(&data[1]) / 1000.0f;
        }
        break;

    case 0x35U:
        if (length == 5U)
        {
            float speed = (float)ZDT_ReadBE16(&data[2]);
            if (firmware == Enum_ZDTMotor_Firmware::X)
            {
                speed /= 10.0f;
            }
            if (data[1] != 0U)
            {
                speed = -speed;
            }
            feedback.speed_rpm = ApplyDirection(speed);
        }
        break;

    case 0x36U:
        if (length == 7U)
        {
            float position = (float)ZDT_ReadBE32(&data[2]);
            position = firmware == Enum_ZDTMotor_Firmware::X
                           ? position / 10.0f
                           : position * 360.0f / 65536.0f;
            if (data[1] != 0U)
            {
                position = -position;
            }
            feedback.position_deg = ApplyDirection(position);
        }
        break;

    case 0x37U:
        if (length == 7U)
        {
            float position_error = (float)ZDT_ReadBE32(&data[2]);
            position_error = firmware == Enum_ZDTMotor_Firmware::X
                                 ? position_error / 100.0f
                                 : position_error * 360.0f / 65536.0f;
            if (data[1] != 0U)
            {
                position_error = -position_error;
            }
            feedback.position_error_deg = ApplyDirection(position_error);
        }
        break;

    case 0x39U:
        if (length == 4U)
        {
            float temperature = data[2];
            if (data[1] != 0U)
            {
                temperature = -temperature;
            }
            feedback.temperature_c = temperature;
        }
        break;

    case 0x3AU:
        if (length == 3U)
        {
            feedback.motor_status = data[1];
            feedback.enabled = (data[1] & 0x01U) != 0U;
            feedback.position_reached = (data[1] & 0x02U) != 0U;
            feedback.stall_detected = (data[1] & 0x04U) != 0U;
            feedback.stall_protected = (data[1] & 0x08U) != 0U;
            feedback.left_limit = (data[1] & 0x10U) != 0U;
            feedback.right_limit = (data[1] & 0x20U) != 0U;
            if (tracked_function == 0xFDU &&
                motion_state == Enum_ZDTMotor_Motion_State::RUNNING &&
                feedback.position_reached)
            {
                motion_state = Enum_ZDTMotor_Motion_State::DONE;
            }
        }
        break;

    case 0x3BU:
        if (length == 3U)
        {
            feedback.homing_status = data[1];
            feedback.encoder_ready = (data[1] & 0x01U) != 0U;
            feedback.calibration_ready = (data[1] & 0x02U) != 0U;
            feedback.homing = (data[1] & 0x04U) != 0U;
            feedback.homing_failed = (data[1] & 0x08U) != 0U;
            feedback.over_temperature = (data[1] & 0x10U) != 0U;
            feedback.over_current = (data[1] & 0x20U) != 0U;
            if (tracked_function == 0x9AU && feedback.homing_failed)
            {
                motion_state = Enum_ZDTMotor_Motion_State::ERROR;
            }
        }
        break;

    default:
        break;
    }
}

void Class_ZDTMotor::FeedbackCallback(FDCAN_HandleTypeDef *callback_hfdcan,
                                      uint32_t id,
                                      uint8_t *data,
                                      uint32_t len,
                                      void *context)
{
    Class_ZDTMotor *motor = (Class_ZDTMotor *)context;
    if (motor == nullptr || data == nullptr || len < 2U || len > FDCAN_MAX_PAYLOAD ||
        motor->hfdcan != callback_hfdcan ||
        id != ((uint32_t)motor->address << 8U))
    {
        return;
    }

    if (!motor->CheckResponseChecksum(data, (uint8_t)len))
    {
        motor->feedback.checksum_error_count++;
        return;
    }

    motor->feedback.online = true;
    motor->feedback.last_rx_timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
    motor->feedback.rx_count++;
    motor->ParseFeedback(data, (uint8_t)len);
}

void Class_ZDTMotor::Process()
{
    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    if (motion_state == Enum_ZDTMotor_Motion_State::WAIT_ACCEPT &&
        now_us - command_timestamp_us >= ZDT_RESPONSE_TIMEOUT_US)
    {
        motion_state = Enum_ZDTMotor_Motion_State::TIMEOUT;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool online = feedback.online;
    const uint64_t last_rx_timestamp_us = feedback.last_rx_timestamp_us;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (online && now_us - last_rx_timestamp_us >= ZDT_OFFLINE_TIMEOUT_US)
    {
        const uint32_t update_primask = __get_PRIMASK();
        __disable_irq();
        if (feedback.last_rx_timestamp_us == last_rx_timestamp_us)
        {
            feedback.online = false;
        }
        __DMB();
        if (update_primask == 0U)
        {
            __enable_irq();
        }
    }
}

Struct_ZDTMotor_Feedback Class_ZDTMotor::GetFeedback() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Struct_ZDTMotor_Feedback snapshot = feedback;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
    return snapshot;
}

Enum_ZDTMotor_Motion_State Class_ZDTMotor::GetMotionState() const
{
    return motion_state;
}

uint8_t Class_ZDTMotor::GetAddress() const
{
    return address;
}
