/**
 * @file zdtmotor.h
 * @brief ZDT X42S 第二代闭环步进电机 CAN 自由协议驱动接口。
 * @details 协议字段、量纲和状态位依据《ZDT_X42S 第二代闭环步进电机用户手册
 *          V1.0.5》的第 4、5、7 章。
 * @author zzm
 */

#ifndef ZDTMOTOR_H
#define ZDTMOTOR_H

#include "bsp_can.h"

#include <stdint.h>

/**
 * @brief 选择 X42S 当前运行的闭环固件协议布局。
 * @note 手册 5.1.2：X 与 Emm 的速度、位置等命令格式不同；该值必须与
 *       0x1A“读取选项参数状态”返回值中的 FwType（bit1）一致。
 */
enum class Enum_ZDTMotor_Firmware : uint8_t
{
    X = 0U,          /*!< FwType=0；支持力矩和梯形位置，速度字段分辨率为 0.1 RPM。 */
    EMM = 1U,        /*!< FwType=1；位置命令使用脉冲数，速度字段分辨率为 1 RPM。 */
    UNKNOWN = 0xFFU, /*!< 尚未通过 0x1A 读到固件类型。 */
};

/** @brief 手册 7.1 定义的自由协议校验方式（本驱动不实现 Modbus-RTU）。 */
enum class Enum_ZDTMotor_Checksum : uint8_t
{
    FIXED_6B = 0U, /*!< 校验字节固定为 0x6B，手册称其便于测试。 */
    XOR = 1U,      /*!< 对逻辑命令中的 Addr、Code 和命令数据逐字节异或。 */
    CRC8 = 2U,     /*!< 对同一逻辑字节序列执行手册附录的 CRC8 查表算法。 */
};

/** @brief 手册 5.3.10/5.3.12 中位置命令的“运动模式”字段。 */
enum class Enum_ZDTMotor_Position_Mode : uint8_t
{
    RELATIVE_TO_TARGET = 0U,  /*!< 相对上一输入目标位置进行相对位置运动。 */
    ABSOLUTE = 1U,            /*!< 相对坐标零点进行绝对位置运动。 */
    RELATIVE_TO_CURRENT = 2U, /*!< 相对当前实时位置进行相对位置运动。 */
};

/** @brief 手册 5.4.2 中 0x9A“触发回零”命令的回零模式字段。 */
enum class Enum_ZDTMotor_Homing_Mode : uint8_t
{
    NEAREST_SINGLE_TURN = 0U,     /*!< 单圈就近回零。 */
    DIRECTIONAL_SINGLE_TURN = 1U, /*!< 单圈方向回零。 */
    COLLISION = 2U,               /*!< 无限位碰撞回零；参数与接线须按手册 3.2 配置。 */
    LIMIT_SWITCH = 3U,            /*!< 限位回零。 */
    ABSOLUTE_ZERO = 4U,           /*!< 回到绝对位置坐标零点。 */
    POWER_OFF_POSITION = 5U,      /*!< 回到上次掉电位置角度。 */
};

/**
 * @brief 驱动对最近一条受跟踪运动命令维护的本地状态。
 * @note 手册 4.1.2/4.2.2：0x02 仅表示命令接收正确；0x9F 才是可选的动作完成
 *       主动返回。位置命令也可通过 0x3A 的 Prf_TF 到位位判定完成。
 */
enum class Enum_ZDTMotor_Motion_State : uint8_t
{
    IDLE = 0U, /*!< 当前没有受跟踪的运动命令。 */
    WAIT_ACCEPT, /*!< 命令已入 CAN 队列，正在等待电机接收确认。 */
    RUNNING,     /*!< 已收到 0x02，电机已接受命令并可能正在运动。 */
    DONE,        /*!< 已收到 0x9F，或轮询到位置到达标志。 */
    ERROR,       /*!< 收到回零零点/限位状态 0x12/0x22、错误码 E2/EE，或回零失败。 */
    TIMEOUT,     /*!< 等待接收确认超时；不代表电机一定没有执行。 */
};

/** @brief 由 CAN 接收中断更新、供任务读取的反馈快照。 */
struct Struct_ZDTMotor_Feedback
{
    bool online = false; /*!< 最近 ZDT_OFFLINE_TIMEOUT_US 内收到过合法反馈。 */
    Enum_ZDTMotor_Firmware reported_firmware = Enum_ZDTMotor_Firmware::UNKNOWN; /*!< 0x1A 返回字节 bit1：0=X，1=Emm。 */
    uint16_t firmware_version = 0U; /*!< 固件版本原始编码，例如 200 表示 V2.0.0。 */
    uint8_t hardware_series = 0U;   /*!< 0x1F 的 HW_Series：0=X 系列，1=Y 系列。 */
    uint8_t hardware_type = 0U;     /*!< 0x1F 返回的 HW_Type 硬件板卡型号原始编码。 */
    uint8_t hardware_version = 0U;  /*!< 0x1F 的 HW_Ver 原始编码，例如 14 表示 V2.0。 */

    float bus_voltage_v = 0.0f;      /*!< 0x24 的 VBus（反接二极管后供电电压），已由 mV 转为 V。 */
    float phase_current_a = 0.0f;    /*!< 0x27 的 CPha（实际工作相电流），已由 mA 转为 A。 */
    float speed_rpm = 0.0f;          /*!< 实时转速，RPM，已应用 reverse 方向。 */
    float position_deg = 0.0f;       /*!< 实时位置，度，已应用 reverse 方向。 */
    float position_error_deg = 0.0f; /*!< 位置误差，度，已应用 reverse 方向。 */
    float temperature_c = 0.0f;      /*!< 0x39 返回的驱动实时温度，单位摄氏度。 */

    uint8_t option_status = 0U; /*!< 功能码 0x1A 返回的选项状态原始位域。 */
    uint8_t motor_status = 0U;  /*!< 功能码 0x3A 返回的电机状态原始位域。 */
    uint8_t homing_status = 0U; /*!< 功能码 0x3B 返回的回零/保护状态原始位域。 */
    bool enabled = false;             /*!< 0x3A Ens_TF(bit0)：0=未使能，1=已使能。 */
    bool position_reached = false;    /*!< 0x3A Prf_TF(bit1)：0=未到达，1=到达输入目标位置。 */
    bool stall_detected = false;      /*!< 0x3A Cgi_TF(bit2)：堵转检测条件是否成立。 */
    bool stall_protected = false;     /*!< 0x3A Cgp_TF(bit3)：是否已触发堵转保护。 */
    bool left_limit = false;          /*!< 0x3A Esi_LF(bit4)：左限位输入引脚为高电平。 */
    bool right_limit = false;         /*!< 0x3A Esi_RF(bit5)：右限位输入引脚为高电平。 */
    bool encoder_ready = false;       /*!< 0x3B Enc_Rdy(bit0)：0=编码器异常，1=正常。 */
    bool calibration_ready = false;   /*!< 0x3B Cal_Rdy(bit1)：0=未校准，1=校准表就绪。 */
    bool homing = false;              /*!< 0x3B Org_SF(bit2)：正在回零。 */
    bool homing_failed = false;       /*!< 0x3B Org_CF(bit3)：回零失败。 */
    bool over_temperature = false;    /*!< 0x3B Otp_TF(bit4)：过热保护已触发。 */
    bool over_current = false;        /*!< 0x3B Ocp_TF(bit5)：过流保护已触发。 */

    uint8_t last_function = 0U;       /*!< 最近一帧合法反馈的功能码。 */
    uint8_t last_response = 0U;       /*!< 最近动作应答码：02/12/22/9F/E2/EE，含义见手册 4.1.2。 */
    uint64_t last_rx_timestamp_us = 0U; /*!< 最近合法反馈时间戳，us。 */
    uint32_t rx_count = 0U;           /*!< 已通过校验的反馈帧总数。 */
    uint32_t checksum_error_count = 0U; /*!< 因校验错误丢弃的帧总数。 */
};

/**
 * @brief 一条 FDCAN 总线上的单台 ZDT X42S 电机实例。
 * @details CAN 使用 29 位扩展帧，ID 为 (Addr << 8) | Packet。控制接口只编码
 *          并提交报文，反馈由 FDCAN RX 中断回调解析；地址 0 保留给广播命令。
 */
class Class_ZDTMotor
{
public:
    /**
     * @brief 初始化实例并注册扩展帧接收回调。
     * @param hfdcan 电机所在的 FDCAN 外设句柄。
     * @param address 电机地址，范围 1~255；地址 0 只用于广播，不能初始化实例。
     * @param firmware 电机当前运行的 X 或 Emm 固件协议布局。
     * @param checksum 与电机 Checksum 菜单一致的自由协议校验方式。
     * @param reverse true 时统一反转控制方向和位置/速度反馈方向。
     * @param pulses_per_revolution Emm 位置命令每机械圈对应的脉冲数；手册默认
     *        1.8 度电机、16 细分时为 3200。
     * @return true 参数合法且回调注册成功；false 初始化失败。
     */
    bool Init(FDCAN_HandleTypeDef *hfdcan,
              uint8_t address,
              Enum_ZDTMotor_Firmware firmware = Enum_ZDTMotor_Firmware::EMM,
              Enum_ZDTMotor_Checksum checksum = Enum_ZDTMotor_Checksum::FIXED_6B,
              bool reverse = false,
              uint32_t pulses_per_revolution = 3200U);

    /** @brief 发送 F3 AB 01 Sync；使能后锁轴，Sync=1 时仅缓存命令。 */
    bool Enable(bool synchronous = false);
    /** @brief 发送 F3 AB 00 Sync；失能后松轴，Sync=1 时仅缓存命令。 */
    bool Disable(bool synchronous = false);
    /** @brief 发送 FE 98 Sync；Sync=0 立即停止，Sync=1 缓存到同步触发。 */
    bool Stop(bool synchronous = false);
    /** @brief 发送 0A 6D，将电机当前位置角度清零。 */
    bool ClearPosition();
    /** @brief 解除堵转、过热和过流保护。 */
    bool ClearProtection();
    /** @brief 发送 08 97，使闭环电机复位重启。 */
    bool Restart();
    /** @brief 发送 9A Mode Sync 触发回零，并跟踪接收、完成或失败状态。 */
    bool TriggerHoming(Enum_ZDTMotor_Homing_Mode mode, bool synchronous = false);
    /** @brief 发送 9C 48，强制中断并退出当前回零过程。 */
    bool AbortHoming();
    /** @brief 以广播地址 0 发送 FF 66，使已缓存命令的电机同步开始执行。 */
    bool TriggerSynchronousMotion();

    /**
     * @brief 发送 F6 速度模式命令。
     * @param speed_rpm 有符号目标转速，正负号决定方向，单位 RPM。
     * @param acceleration X 固件为 0~65535 RPM/s；Emm 固件为 0~255 档位，
     *        0 表示直接启动，非 0 时数值越大加减速越快。
     * @param synchronous true 时只缓存，等待 TriggerSynchronousMotion()。
     * @return true 命令成功进入 CAN 队列；false 参数越界或队列提交失败。
     */
    bool SetSpeed(float speed_rpm,
                  uint16_t acceleration,
                  bool synchronous = false);
    /**
     * @brief 发送 FD 位置命令：X 为梯形曲线位置，Emm 为脉冲数位置。
     * @param position_deg 有符号目标位置，单位度；符号决定运动方向。
     * @param speed_rpm 最大速度，单位 RPM，必须为非负值。
     * @param acceleration X 固件为加速加速度（RPM/s）；Emm 固件为 0~255 档位。
     * @param deceleration X 固件为减速加速度（RPM/s）；Emm 报文没有该字段。
     * @param mode 目标位置的参考坐标方式。
     * @param synchronous true 时只缓存，等待广播同步触发。
     * @return true 命令成功进入 CAN 队列；false 参数越界或队列提交失败。
     */
    bool SetPosition(float position_deg,
                     float speed_rpm,
                     uint16_t acceleration,
                     uint16_t deceleration,
                     Enum_ZDTMotor_Position_Mode mode = Enum_ZDTMotor_Position_Mode::RELATIVE_TO_CURRENT,
                     bool synchronous = false);
    /**
     * @brief 发送 X 固件 F5 力矩模式命令（实际字段为相电流）。
     * @param current_a 有符号电流，单位 A，范围 -5~5 A；符号映射为 CW/CCW。
     * @param current_slope_ma_s 电流斜率，范围 0~65535 mA/s。
     * @param synchronous true 时只缓存，等待广播同步触发。
     * @return true 命令成功入队；false 当前不是 X 固件、参数无效或入队失败。
     */
    bool SetTorque(float current_a,
                   uint16_t current_slope_ma_s,
                   bool synchronous = false);

    /** @brief 发送 1F，读取 FW_Ver、HW_Series、HW_Type 和 HW_Ver。 */
    bool ReadVersion();
    /** @brief 发送 1A，读取 MotType、FwType、CtrMode、MotDir 等选项位。 */
    bool ReadOptionStatus();
    /** @brief 发送 24，读取单位为 mV 的总线电压 VBus。 */
    bool ReadBusVoltage();
    /** @brief 发送 27，读取单位为 mA 的实际工作相电流 CPha。 */
    bool ReadPhaseCurrent();
    /** @brief 发送 35，读取实时转速（Emm 为 RPM，X 为 0.1 RPM）。 */
    bool ReadSpeed();
    /** @brief 发送 36，读取实时位置（Emm 为 65536/圈，X 为 0.1 度）。 */
    bool ReadPosition();
    /** @brief 发送 37，读取位置误差（Emm 为 65536/圈，X 为 0.01 度）。 */
    bool ReadPositionError();
    /** @brief 发送 39，读取驱动实时温度。 */
    bool ReadTemperature();
    /** @brief 发送 3A，读取使能、到位、堵转、限位输入及掉电状态位。 */
    bool ReadMotorStatus();
    /** @brief 发送 3B，读取编码器、校准、回零、过热和过流状态位。 */
    bool ReadHomingStatus();
    /**
     * @brief 发送 11 18 Function Period，配置定时返回系统参数。
     * @param function 手册 5.5 中的读取功能码，例如 0x36 表示实时位置。
     * @param period_ms 0~65535 ms；0 表示停止该功能码的定时返回。
     */
    bool ConfigurePeriodicFeedback(uint8_t function, uint16_t period_ms);
    /** @brief 配置 0x35 实时转速周期返回；period_ms 为 0 时停止返回。 */
    bool ConfigureSpeedFeedbackPeriod(uint16_t period_ms = 10U);
    /** @brief 配置 0x36 实时位置周期返回；period_ms 为 0 时停止返回。 */
    bool ConfigurePositionFeedbackPeriod(uint16_t period_ms = 10U);

    /** @brief 周期维护接收确认超时和在线状态，建议在任务中定期调用。 */
    void Process();
    /** @brief 在短临界区内复制并返回一致的反馈快照。 */
    Struct_ZDTMotor_Feedback GetFeedback() const;
    /** @brief 返回最近受跟踪运动命令的状态。 */
    Enum_ZDTMotor_Motion_State GetMotionState() const;
    /** @brief 返回本实例绑定的电机地址。 */
    uint8_t GetAddress() const;

private:
    /** @brief BSP CAN 扩展帧接收回调，在 FDCAN 中断上下文执行。 */
    static void FeedbackCallback(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);

    /**
     * @brief 计算校验，并按手册 4.2.1 的 CAN 扩展帧规则分包提交。
     * @note ID=(target_address<<8)|Packet，Packet 从 0 递增；每包 data[0] 重复
     *       功能码，其余最多 7 字节连续承载命令数据和末尾校验码。
     */
    bool SubmitCommand(uint8_t target_address,
                       uint8_t function,
                       const uint8_t *body,
                       uint8_t body_length,
                       bool track_motion);
    /** @brief 校验 CAN 返回数据；计算 XOR/CRC8 时把扩展 ID 中的 Addr 补回首字节。 */
    bool CheckResponseChecksum(const uint8_t *data, uint8_t length) const;
    /** @brief 对逻辑命令 [Addr, Function, Body...] 计算配置的校验字节。 */
    uint8_t CalculateCommandChecksum(uint8_t target_address,
                                     uint8_t function,
                                     const uint8_t *body,
                                     uint8_t body_length) const;
    /** @brief 以反射多项式 0x8C 实现手册 7.1 查表 CRC8 的单字节更新。 */
    static uint8_t UpdateCRC8(uint8_t crc, uint8_t data);
    /** @brief 按功能码解析合法反馈并更新 feedback 与运动状态。 */
    void ParseFeedback(const uint8_t *data, uint8_t length);
    /** @brief 根据 reverse 配置统一转换应用层方向。 */
    float ApplyDirection(float value) const;

    FDCAN_HandleTypeDef *hfdcan = nullptr; /*!< 绑定的 FDCAN 总线。 */
    uint8_t address = 0U;                 /*!< 电机节点地址，范围 1~255。 */
    Enum_ZDTMotor_Firmware firmware = Enum_ZDTMotor_Firmware::EMM; /*!< 本地采用的协议字段布局。 */
    Enum_ZDTMotor_Checksum checksum = Enum_ZDTMotor_Checksum::FIXED_6B; /*!< 当前校验方式。 */
    bool reverse = false;                 /*!< 应用层正方向是否与电机协议正方向相反。 */
    uint32_t pulses_per_revolution = 3200U; /*!< Emm 位置命令一机械圈对应的脉冲数。 */
    volatile Enum_ZDTMotor_Motion_State motion_state = Enum_ZDTMotor_Motion_State::IDLE; /*!< 中断和任务共享的运动状态。 */
    volatile uint8_t tracked_function = 0U; /*!< 当前等待应答/完成的运动功能码。 */
    volatile uint64_t command_timestamp_us = 0U; /*!< 当前受跟踪命令的提交时间。 */
    Struct_ZDTMotor_Feedback feedback;     /*!< 中断更新、任务读取的反馈数据。 */
};

#endif
