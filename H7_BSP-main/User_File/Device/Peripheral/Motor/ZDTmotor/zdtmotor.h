/**
 * @file zdtmotor.h
 * @brief ZDT X42S 第二代闭环步进电机 CAN 自由协议驱动。
 * @author zzm
 */

#ifndef ZDTMOTOR_H
#define ZDTMOTOR_H

#include "bsp_can.h"

#include <stdint.h>

enum class Enum_ZDTMotor_Firmware : uint8_t
{
    X = 0U,
    EMM = 1U,
    UNKNOWN = 0xFFU,
};

enum class Enum_ZDTMotor_Checksum : uint8_t
{
    FIXED_6B = 0U,
    XOR = 1U,
    CRC8 = 2U,
};

enum class Enum_ZDTMotor_Position_Mode : uint8_t
{
    RELATIVE_TO_TARGET = 0U,
    ABSOLUTE = 1U,
    RELATIVE_TO_CURRENT = 2U,
};

enum class Enum_ZDTMotor_Homing_Mode : uint8_t
{
    NEAREST_SINGLE_TURN = 0U,
    DIRECTIONAL_SINGLE_TURN = 1U,
    COLLISION = 2U,
    LIMIT_SWITCH = 3U,
    ABSOLUTE_ZERO = 4U,
    POWER_OFF_POSITION = 5U,
};

enum class Enum_ZDTMotor_Motion_State : uint8_t
{
    IDLE = 0U,
    WAIT_ACCEPT,
    RUNNING,
    DONE,
    ERROR,
    TIMEOUT,
};

struct Struct_ZDTMotor_Feedback
{
    bool online = false;
    Enum_ZDTMotor_Firmware reported_firmware = Enum_ZDTMotor_Firmware::UNKNOWN;
    uint16_t firmware_version = 0U;
    uint8_t hardware_series = 0U;
    uint8_t hardware_type = 0U;
    uint8_t hardware_version = 0U;

    float bus_voltage_v = 0.0f;
    float phase_current_a = 0.0f;
    float speed_rpm = 0.0f;
    float position_deg = 0.0f;
    float position_error_deg = 0.0f;
    float temperature_c = 0.0f;

    uint8_t option_status = 0U;
    uint8_t motor_status = 0U;
    uint8_t homing_status = 0U;
    bool enabled = false;
    bool position_reached = false;
    bool stall_detected = false;
    bool stall_protected = false;
    bool left_limit = false;
    bool right_limit = false;
    bool encoder_ready = false;
    bool calibration_ready = false;
    bool homing = false;
    bool homing_failed = false;
    bool over_temperature = false;
    bool over_current = false;

    uint8_t last_function = 0U;
    uint8_t last_response = 0U;
    uint64_t last_rx_timestamp_us = 0U;
    uint32_t rx_count = 0U;
    uint32_t checksum_error_count = 0U;
};

class Class_ZDTMotor
{
public:
    bool Init(FDCAN_HandleTypeDef *hfdcan,
              uint8_t address,
              Enum_ZDTMotor_Firmware firmware = Enum_ZDTMotor_Firmware::EMM,
              Enum_ZDTMotor_Checksum checksum = Enum_ZDTMotor_Checksum::FIXED_6B,
              bool reverse = false,
              uint32_t pulses_per_revolution = 3200U);

    bool Enable(bool synchronous = false);
    bool Disable(bool synchronous = false);
    bool Stop(bool synchronous = false);
    bool ClearPosition();
    bool ClearProtection();
    bool Restart();
    bool TriggerHoming(Enum_ZDTMotor_Homing_Mode mode, bool synchronous = false);
    bool AbortHoming();
    bool TriggerSynchronousMotion();

    bool SetSpeed(float speed_rpm,
                  uint16_t acceleration,
                  bool synchronous = false);
    bool SetPosition(float position_deg,
                     float speed_rpm,
                     uint16_t acceleration,
                     uint16_t deceleration,
                     Enum_ZDTMotor_Position_Mode mode = Enum_ZDTMotor_Position_Mode::RELATIVE_TO_CURRENT,
                     bool synchronous = false);
    bool SetTorque(float current_a,
                   uint16_t current_slope_ma_s,
                   bool synchronous = false);

    bool ReadVersion();
    bool ReadOptionStatus();
    bool ReadBusVoltage();
    bool ReadPhaseCurrent();
    bool ReadSpeed();
    bool ReadPosition();
    bool ReadPositionError();
    bool ReadTemperature();
    bool ReadMotorStatus();
    bool ReadHomingStatus();
    bool ConfigurePeriodicFeedback(uint8_t function, uint16_t period_ms);

    void Process();
    Struct_ZDTMotor_Feedback GetFeedback() const;
    Enum_ZDTMotor_Motion_State GetMotionState() const;
    uint8_t GetAddress() const;

private:
    static void FeedbackCallback(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);

    bool SubmitCommand(uint8_t target_address,
                       uint8_t function,
                       const uint8_t *body,
                       uint8_t body_length,
                       bool track_motion);
    bool CheckResponseChecksum(const uint8_t *data, uint8_t length) const;
    uint8_t CalculateCommandChecksum(uint8_t target_address,
                                     uint8_t function,
                                     const uint8_t *body,
                                     uint8_t body_length) const;
    static uint8_t UpdateCRC8(uint8_t crc, uint8_t data);
    void ParseFeedback(const uint8_t *data, uint8_t length);
    float ApplyDirection(float value) const;

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint8_t address = 0U;
    Enum_ZDTMotor_Firmware firmware = Enum_ZDTMotor_Firmware::EMM;
    Enum_ZDTMotor_Checksum checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
    bool reverse = false;
    uint32_t pulses_per_revolution = 3200U;
    volatile Enum_ZDTMotor_Motion_State motion_state = Enum_ZDTMotor_Motion_State::IDLE;
    volatile uint8_t tracked_function = 0U;
    volatile uint64_t command_timestamp_us = 0U;
    Struct_ZDTMotor_Feedback feedback;
};

#endif
