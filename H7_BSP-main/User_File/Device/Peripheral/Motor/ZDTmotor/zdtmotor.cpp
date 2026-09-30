/**
 * @file zdtmotor.cpp
 * @brief ZDT X42S 第二代闭环步进电机 CAN 命令编码与反馈解析。
 * @details 注释中的命令格式和量纲来自用户手册 V1.0.5 第 4、5、7 章；普通
 *          CAN 命令的数据区不含 Addr，Addr 编入 29 位扩展 ID；AA 多电机
 *          命令的内嵌 Addr 属于协议数据，外层扩展 ID 仍使用广播地址 0。
 * @author zzm
 */

#include "zdtmotor.h"

#include "sys_timestamp.h"

#include <cmath>
#include <cstring>

/** 单条 FD(X) 命令体为 13 字节；多电机 AA 命令需容纳多条内嵌命令，预留 64 字节。 */
static constexpr uint8_t ZDT_MAX_COMMAND_BODY = 64U;
/** 每个 Classic CAN 分包用 1 字节重复 Code，最多承载 7 个后续逻辑字节。 */
static constexpr uint8_t ZDT_MAX_COMMAND_FRAMES = 10U;
/** 本地等待 0x02“命令接收正确”应答的超时；不是手册规定的运动超时。 */
static constexpr uint64_t ZDT_RESPONSE_TIMEOUT_US = 50000U;
/** 驱动本地在线判定窗口；手册没有规定该数值。 */
static constexpr uint64_t ZDT_OFFLINE_TIMEOUT_US = 500000U;

/** @brief 从手册表格所示的高字节在前字段读取 uint16_t。 */
static uint16_t ZDT_ReadBE16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

/** @brief 从手册表格所示的高字节在前字段读取 uint32_t。 */
static uint32_t ZDT_ReadBE32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           data[3];
}

/** @brief 按高字节在前写入两字节协议字段。 */
static void ZDT_WriteBE16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

/** @brief 按高字节在前写入四字节协议字段。 */
static void ZDT_WriteBE32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

/** @brief 检查速度等只允许非负数值的公开接口参数。 */
static bool ZDT_IsFiniteNonNegative(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}

/**
 * @brief 判断 Code 是否为本驱动发送的动作/配置命令。
 * @details 动作短应答的数据字节可能为 02/12/22/9F/E2/EE；读取反馈的首个
 *          数据字节也可能碰巧等于这些值，因此需先按 Code 区分。
 */
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
    case 0xAAU:
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

/**
 * @brief 初始化本地协议配置，并注册 Packet=0 的扩展帧接收回调。
 * @details 手册 4.2 规定 ID=(Addr<<8)|Packet。当前驱动支持的短应答和读取
 *          反馈均落在 Packet=0，因此监听 address<<8；Addr=0 仅用于广播。
 */
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

    /* 手册 4.2.1：CAN 通讯帧类型固定为扩展帧。 */
    return BSP_CAN_RegisterCallbackEx((uint32_t)address << 8U,
                                      CAN_ID_EXTENDED,
                                      hfdcan,
                                      FeedbackCallback,
                                      this);
}

/**
 * @brief 逐位实现手册 7.1 中 crc8Table 的一次状态更新。
 * @note 表中首行 00,5E,BC,E2... 对应反射多项式 0x8C；逐位法与查表法等价。
 */
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

/** @brief 对逻辑字节序列 [Addr, Code, Data...] 计算末尾校验字节。 */
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

    /* 手册示例令 crc8=p[0]，因此以 Addr 为初值，再更新 Code 和 Data。 */
    uint8_t result = target_address;
    result = UpdateCRC8(result, function);
    for (uint8_t index = 0U; index < body_length; index++)
    {
        result = UpdateCRC8(result, body[index]);
    }
    return result;
}

/** @brief 校验 CAN 返回数据；XOR/CRC8 计算时将 ID 中的 Addr 补回逻辑帧首。 */
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

/**
 * @brief 按手册 4.2.1 将一条逻辑命令拆成 Classic CAN 扩展帧并连续入队。
 *
 * CAN 数据区不发送 Addr，扩展 ID 使用 `(Addr << 8) | Packet`，Packet 从 0
 * 开始；每包 data[0] 都重复 Code，data[1..7] 顺序承载 Data 和最终校验字节。
 * `CAN_Tx_SubmitBatch()` 保证同一长命令的分包不会被其他任务的消息穿插。
 */
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

    /* tail 对应逻辑命令中 Code 之后的连续字节：Data + Checksum。 */
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

    /* 手册要求每包第一个数据字节重复 Code，余下空间按顺序切片 tail。 */
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
        /* 接收回调也会更新状态；关中断后再建立本次命令的跟踪上下文。 */
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

/** @brief 手册 5.3.2：编码 F3 AB 01 Sync，使能电机（锁轴）。 */
bool Class_ZDTMotor::Enable(bool synchronous)
{
    const uint8_t body[] = {0xABU, 0x01U, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    return SubmitCommand(address, 0xF3U, body, sizeof(body), false);
}

/** @brief 手册 5.3.2：编码 F3 AB 00 Sync，不使能电机（松轴）。 */
bool Class_ZDTMotor::Disable(bool synchronous)
{
    const uint8_t body[] = {0xABU, 0x00U, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    return SubmitCommand(address, 0xF3U, body, sizeof(body), false);
}

/** @brief 手册 5.3.15：编码 FE 98 Sync；Sync=0 立即停止，Sync=1 先缓存。 */
bool Class_ZDTMotor::Stop(bool synchronous)
{
    const uint8_t body[] = {0x98U, (uint8_t)(synchronous ? 0x01U : 0x00U)};
    if (!SubmitCommand(address, 0xFEU, body, sizeof(body), false))
    {
        return false;
    }

    /* 停止命令入队后终止对上一条运动命令的本地状态跟踪。 */
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

/** @brief 手册 5.2.3：编码 0A 6D，将当前位置角度清零。 */
bool Class_ZDTMotor::ClearPosition()
{
    const uint8_t body[] = {0x6DU};
    return SubmitCommand(address, 0x0AU, body, sizeof(body), false);
}

/** @brief 手册 5.2.4：编码 0E 52，解除堵转、过热和过流保护。 */
bool Class_ZDTMotor::ClearProtection()
{
    const uint8_t body[] = {0x52U};
    return SubmitCommand(address, 0x0EU, body, sizeof(body), false);
}

/** @brief 手册 5.2.2：编码 08 97，使闭环电机复位重启。 */
bool Class_ZDTMotor::Restart()
{
    const uint8_t body[] = {0x97U};
    return SubmitCommand(address, 0x08U, body, sizeof(body), false);
}

/** @brief 手册 5.4.2：编码 9A Mode Sync，触发选定回零模式。 */
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

/** @brief 手册 5.4.3：编码 9C 48，强制中断并退出回零。 */
bool Class_ZDTMotor::AbortHoming()
{
    const uint8_t body[] = {0x48U};
    return SubmitCommand(address, 0x9CU, body, sizeof(body), false);
}

/** @brief 手册 5.3.16：以广播 ID 0x0000 发送 FF 66，触发已缓存命令。 */
bool Class_ZDTMotor::TriggerSynchronousMotion()
{
    const uint8_t body[] = {0x66U};
    return SubmitCommand(0U, 0xFFU, body, sizeof(body), false);
}

/**
 * @brief 按固件类型编码 F6 速度命令体，不包含 Addr、Code 和校验字节。
 * @details X：Dir + Acc(RPM/s)[2] + Speed(0.1 RPM)[2] + Sync；Emm：
 *          Dir + Speed(RPM)[2] + AccLevel[1] + Sync。
 */
bool Class_ZDTMotor::EncodeSpeedBody(float speed_rpm,
                                     uint16_t acceleration,
                                     bool synchronous,
                                     uint8_t *body,
                                     uint8_t &body_length) const
{
    body_length = 0U;
    if (body == nullptr || !std::isfinite(speed_rpm))
    {
        return false;
    }

    /* 协议规定 00=CW、01=CCW；reverse 统一翻转应用层与协议层的方向定义。 */
    const float directed_speed = reverse ? -speed_rpm : speed_rpm;
    const uint8_t direction = directed_speed < 0.0f ? 0x01U : 0x00U;
    const float magnitude = std::fabs(directed_speed);
    body[0] = direction;

    if (firmware == Enum_ZDTMotor_Firmware::X)
    {
        if (magnitude > 3000.0f)
        {
            return false;
        }
        ZDT_WriteBE16(&body[1], acceleration);
        /* X 速度字段分辨率为 0.1 RPM，范围 0~30000（0~3000.0 RPM）。 */
        ZDT_WriteBE16(&body[3], (uint16_t)(magnitude * 10.0f + 0.5f));
        body[5] = synchronous ? 0x01U : 0x00U;
        body_length = 6U;
        return true;
    }

    if (magnitude > 3000.0f || acceleration > 255U)
    {
        return false;
    }
    /* Emm 速度为整数 RPM；Acc=0 直接启动，非 0 时数值越大加减速越快。 */
    ZDT_WriteBE16(&body[1], (uint16_t)(magnitude + 0.5f));
    body[3] = (uint8_t)acceleration;
    body[4] = synchronous ? 0x01U : 0x00U;
    body_length = 5U;
    return true;
}

/**
 * @brief 按手册 5.3.1 将多台电机的 F6 速度命令拼成一条 AA 多电机命令。
 *
 * 逻辑帧为 `00 AA Length[2] (Addr Code Data Checksum)... Checksum`；Length
 * 是整个逻辑帧的总字节数。CAN 分包由 SubmitCommand 统一按 `(Addr<<8)|Packet`
 * 处理：外层 Addr=0，Packet 从 0 开始，每包 data[0] 重复 Code=AA。
 * 内嵌速度命令的同步标志固定为 0，因为 AA 命令本身已使所有内嵌命令同时开始。
 */
bool Class_ZDTMotor::SetMultiMotorSpeeds(const Class_ZDTMotor *motor_group,
                                         const float *speed_rpm,
                                         const uint16_t *acceleration,
                                         uint8_t motor_count)
{
    if (motor_group == nullptr || speed_rpm == nullptr ||
        acceleration == nullptr || motor_count == 0U || address == 0U || hfdcan == nullptr)
    {
        return false;
    }

    uint8_t body[ZDT_MAX_COMMAND_BODY]{};
    uint16_t body_length = 2U; /* 先保留 AA 命令的 2 字节总长度字段。 */

    for (uint8_t i = 0U; i < motor_count; ++i)
    {
        if (motor_group[i].hfdcan != hfdcan ||
            motor_group[i].checksum != checksum ||
            motor_group[i].address == 0U)
        {
            return false;
        }

        for (uint8_t j = 0U; j < i; ++j)
        {
            if (motor_group[j].address == motor_group[i].address)
            {
                return false;
            }
        }

        uint8_t command_body[6]{};
        uint8_t command_body_length = 0U;
        if (!motor_group[i].EncodeSpeedBody(speed_rpm[i],
                                            acceleration[i],
                                            false,
                                            command_body,
                                            command_body_length))
        {
            return false;
        }

        /* 内嵌命令包含 Addr、Code、数据体和各自的校验字节。 */
        const uint8_t inner_length =
            (uint8_t)(1U + 1U + command_body_length + 1U);
        if ((uint16_t)body_length + inner_length > sizeof(body))
        {
            return false;
        }

        body[body_length++] = motor_group[i].address;
        body[body_length++] = 0xF6U;
        std::memcpy(&body[body_length], command_body, command_body_length);
        body_length = (uint8_t)(body_length + command_body_length);
        body[body_length++] = motor_group[i].CalculateCommandChecksum(
            motor_group[i].address, 0xF6U, command_body, command_body_length);
    }

    /* 总长度包含 Addr、Code、长度字段、内嵌命令和最后一条校验码。 */
    ZDT_WriteBE16(&body[0], (uint16_t)(body_length + 3U));
    return SubmitCommand(0U, 0xAAU, body, (uint8_t)body_length, false);
}

/**
 * @brief 发送单台电机速度命令，编码规则见 EncodeSpeedBody()。
 */
bool Class_ZDTMotor::SetSpeed(float speed_rpm,
                              uint16_t acceleration,
                              bool synchronous)
{
    uint8_t body[6]{};
    uint8_t body_length = 0U;
    if (!EncodeSpeedBody(speed_rpm, acceleration, synchronous, body, body_length))
    {
        return false;
    }
    return SubmitCommand(address, 0xF6U, body, body_length, true);
}

/**
 * @brief 根据固件类型编码手册 5.3.10/5.3.12 的 FD 位置命令。
 * @details X 使用加/减速度、0.1 RPM 最大速度和 0.1 度位置；Emm 使用整数
 *          RPM、加速度档位和脉冲数，报文中没有 deceleration 字段。
 */
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
        /* X 的位置绝对值以 0.1 度编码到四字节无符号字段。 */
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

    /* Emm 的四字节位置字段为脉冲数；手册默认 1.8 度、16 细分即 3200/圈。 */
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

/**
 * @brief 编码手册 5.3.3 的 X 固件 F5 力矩模式命令。
 * @details 字段为 Sign(00=CW/01=CCW) + Slope(mA/s)[2] + Current(mA)[2]
 *          + Sync；电流范围为 0~5000 mA。
 */
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

/** @brief 手册 5.5.2：发送 1F，读取固件版本和硬件版本。 */
bool Class_ZDTMotor::ReadVersion()
{
    return SubmitCommand(address, 0x1FU, nullptr, 0U, false);
}

/** @brief 手册 5.6.4：发送 1A，读取选项参数状态位。 */
bool Class_ZDTMotor::ReadOptionStatus()
{
    return SubmitCommand(address, 0x1AU, nullptr, 0U, false);
}

/** @brief 手册 5.5.4：发送 24，读取反接二极管后的总线电压。 */
bool Class_ZDTMotor::ReadBusVoltage()
{
    return SubmitCommand(address, 0x24U, nullptr, 0U, false);
}

/** @brief 手册 5.5.6：发送 27，读取电机实际工作相电流。 */
bool Class_ZDTMotor::ReadPhaseCurrent()
{
    return SubmitCommand(address, 0x27U, nullptr, 0U, false);
}

/** @brief 手册 5.5.11：发送 35，读取电机实时转速。 */
bool Class_ZDTMotor::ReadSpeed()
{
    return SubmitCommand(address, 0x35U, nullptr, 0U, false);
}

/** @brief 手册 5.5.13：发送 36，读取电机实时位置。 */
bool Class_ZDTMotor::ReadPosition()
{
    return SubmitCommand(address, 0x36U, nullptr, 0U, false);
}

/** @brief 手册 5.5.14：发送 37，读取电机位置误差。 */
bool Class_ZDTMotor::ReadPositionError()
{
    return SubmitCommand(address, 0x37U, nullptr, 0U, false);
}

/** @brief 手册 5.5.12：发送 39，读取驱动实时温度。 */
bool Class_ZDTMotor::ReadTemperature()
{
    return SubmitCommand(address, 0x39U, nullptr, 0U, false);
}

/** @brief 手册 5.5.15：发送 3A，读取电机状态标志。 */
bool Class_ZDTMotor::ReadMotorStatus()
{
    return SubmitCommand(address, 0x3AU, nullptr, 0U, false);
}

/** @brief 手册 5.4.4：发送 3B，读取回零状态标志。 */
bool Class_ZDTMotor::ReadHomingStatus()
{
    return SubmitCommand(address, 0x3BU, nullptr, 0U, false);
}

/** @brief 手册 5.5.1：编码 11 18 Function Period(ms)，0 ms 表示停止返回。 */
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

/** @brief 手册 5.5.1/5.5.11：配置实时转速周期返回，默认 10 ms。 */
bool Class_ZDTMotor::ConfigureSpeedFeedbackPeriod(uint16_t period_ms)
{
    return ConfigurePeriodicFeedback(0x35U, period_ms);
}

/** @brief 手册 5.5.1/5.5.13：配置实时位置周期返回，默认 10 ms。 */
bool Class_ZDTMotor::ConfigurePositionFeedbackPeriod(uint16_t period_ms)
{
    return ConfigurePeriodicFeedback(0x36U, period_ms);
}

/** @brief 按 Init() 的 reverse 配置将协议方向映射到应用层方向。 */
float Class_ZDTMotor::ApplyDirection(float value) const
{
    return reverse ? -value : value;
}

/**
 * @brief 解析已通过地址和校验检查的 Packet=0 返回帧。
 * @note 本函数在 FDCAN 接收中断中执行，只更新内存，不调用阻塞接口。
 */
void Class_ZDTMotor::ParseFeedback(const uint8_t *data, uint8_t length)
{
    const uint8_t function = data[0];
    feedback.last_function = function;

    /* CAN 短应答数据区为 Code + Status + Checksum；Addr 位于扩展 ID。 */
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
        /* 1F + FW_Ver[2] + (HW_Series[4bit]|HW_Type[4bit]) + HW_Ver + Checksum。 */
        if (length == 6U)
        {
            feedback.firmware_version = ZDT_ReadBE16(&data[1]);
            feedback.hardware_series = (data[3] >> 4U) & 0x0FU;
            feedback.hardware_type = data[3] & 0x0FU;
            feedback.hardware_version = data[4];
        }
        break;

    case 0x1AU:
        /* 1A 状态：bit0 MotType，bit1 FwType，bit2 CtrMode，bit4 MotDir 等。 */
        if (length == 3U)
        {
            feedback.option_status = data[1];
            feedback.reported_firmware = (data[1] & 0x02U) != 0U
                                             ? Enum_ZDTMotor_Firmware::EMM
                                             : Enum_ZDTMotor_Firmware::X;
        }
        break;

    case 0x24U:
        /* VBus 单位为 mV；对外反馈换算为 V。 */
        if (length == 4U)
        {
            feedback.bus_voltage_v = (float)ZDT_ReadBE16(&data[1]) / 1000.0f;
        }
        break;

    case 0x27U:
        /* CPha 为电机实际工作相电流，单位 mA；对外反馈换算为 A。 */
        if (length == 4U)
        {
            feedback.phase_current_a = (float)ZDT_ReadBE16(&data[1]) / 1000.0f;
        }
        break;

    case 0x35U:
        if (length == 5U)
        {
            /* 35 返回 Sign(00=正/01=负) + Speed[2]；X 单位 0.1 RPM，Emm 为 RPM。 */
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
            /* 36 返回 Sign(00=正/01=负)；X 为 0.1 度，Emm 为 65536/圈。 */
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
            /* 37 返回 Sign(00=正/01=负)；X 为 0.01 度，Emm 为 65536/圈。 */
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
            /* 手册 5.5.12 明确温度 Sign：00=负、01=正；当前实现却按非零为负。 */
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
            /* 3A：bit7 Oac_TF，bit5/4 右/左限位电平，bit3/2 堵转保护/堵转，
             *     bit1 Prf_TF，bit0 Ens_TF；bit6 保留。 */
            feedback.motor_status = data[1];
            feedback.enabled = (data[1] & 0x01U) != 0U;
            feedback.position_reached = (data[1] & 0x02U) != 0U;
            feedback.stall_detected = (data[1] & 0x04U) != 0U;
            feedback.stall_protected = (data[1] & 0x08U) != 0U;
            feedback.left_limit = (data[1] & 0x10U) != 0U;
            feedback.right_limit = (data[1] & 0x20U) != 0U;
            /* 即使未配置 0x9F 到位主动返回，也可用 Prf_TF 完成本地状态跟踪。 */
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
            /* 3B：bit5 Ocp_TF，bit4 Otp_TF，bit3 Org_CF，bit2 Org_SF，
             *     bit1 Cal_Rdy，bit0 Enc_Rdy；bit7/6 保留。 */
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

/**
 * @brief 接收本实例 Packet=0 扩展帧，校验通过后更新反馈。
 * @details BSP 以 (FDCAN, 帧类型, ID) 精确分发；这里再次检查实例总线和地址，
 *          防止错误上下文污染实例数据。校验失败的帧只累计计数，不更新在线状态。
 */
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

    /* online 是驱动本地健康状态，只允许通过校验的反馈刷新。 */
    motor->feedback.online = true;
    motor->feedback.last_rx_timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
    motor->feedback.rx_count++;
    motor->ParseFeedback(data, (uint8_t)len);
}

/**
 * @brief 在任务上下文维护本地应答超时和反馈在线状态。
 * @note 50 ms 只用于等待 0x02 接收确认；0x9F 或 Prf_TF 才表示位置动作完成，
 *       机械运动时间不能用 CAN 接收确认超时替代。
 */
void Class_ZDTMotor::Process()
{
    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    if (motion_state == Enum_ZDTMotor_Motion_State::WAIT_ACCEPT &&
        now_us - command_timestamp_us >= ZDT_RESPONSE_TIMEOUT_US)
    {
        motion_state = Enum_ZDTMotor_Motion_State::TIMEOUT;
    }

    /* Cortex-M7 读取 64 位时间戳并非天然原子操作，需与接收中断互斥。 */
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
        /* 进入临界区后复核快照，避免并发新反馈被误判为离线。 */
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

/** @brief 在短临界区复制完整反馈，避免任务读到中断更新一半的数据。 */
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

/** @brief 返回最近一条 F5/F6/FD/9A 受跟踪命令的本地状态。 */
Enum_ZDTMotor_Motion_State Class_ZDTMotor::GetMotionState() const
{
    return motion_state;
}

/** @brief 获取实例绑定的电机地址。 */
uint8_t Class_ZDTMotor::GetAddress() const
{
    return address;
}
