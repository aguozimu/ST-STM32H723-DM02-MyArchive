# ZDT X42S CAN 驱动

本目录提供 ZDT X42S 第二代闭环步进电机的 CAN 自由协议驱动。实现依据随任务提供的 `README.md` 和《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5》核对，支持经典 CAN 29 位扩展帧、长命令分包、X/Emm 两种固件以及固定 `0x6B`、XOR、CRC-8 三种校验。

## 工程接入

驱动不绑定具体 FDCAN 外设，也不会修改总线波特率或创建任务。使用前由应用层选择并配置总线：

- 波特率必须与电机一致；X42S 出厂默认通常为 `500 kbit/s`。
- 帧格式：经典 CAN 扩展帧，不使用 CAN FD。
- 电机地址范围为 `1~255`；例如地址 1 的扩展帧基础 ID 为 `0x0100`。
- 固件可以是 `Emm` 或 `X`，必须与 `Init()` 参数一致。
- 校验默认通常为固定 `0x6B`，也可以选择 XOR 或 CRC-8。

底层 `bsp_can` 同时接收标准帧和扩展帧。原有 DJI、DM、QDrive 驱动的零初始化消息仍是标准帧；ZDT 驱动显式设置 `CAN_ID_EXTENDED`。长命令通过 `CAN_Tx_SubmitBatch()` 一次性连续入队，避免其他任务把报文插进同一条 ZDT 多包命令。

本驱动只提供设备接口，不创建 RTOS Task、不定义全局电机实例，也不自动发送任何读取、使能或运动命令。应用层应自行创建 `Class_ZDTMotor` 实例，并在已有任务中安排初始化、状态轮询和控制。不要在中断中调用发送接口；长命令批量入队使用 RTOS 互斥锁，只允许任务上下文调用。

## 初始化

```cpp
static Class_ZDTMotor motor;

bool ok = motor.Init(&hfdcan,
                     1,
                     Enum_ZDTMotor_Firmware::EMM,
                     Enum_ZDTMotor_Checksum::FIXED_6B,
                     false,
                     3200U);
```

参数依次为 FDCAN 句柄、地址、固件、校验、反向安装标志和 Emm 每圈脉冲数。地址必须为 `1~255`；地址 0 只用于广播。

## 基础命令

```cpp
motor.ReadVersion();
motor.ReadOptionStatus();
motor.Enable();
motor.Stop();
motor.Disable();
motor.ClearPosition();
motor.ClearProtection();
motor.Restart();
```

`ReadOptionStatus()` 的返回值会写入 `feedback.reported_firmware`。发送运动命令前，应确认它与 `Init()` 传入的固件一致；X 和 Emm 的 `F6`、`FD` 字段及单位不同，不能混用。

## 速度和位置

统一接口按初始化时的固件自动编码：

```cpp
// Emm：加速度是 0~255 档位；速度单位 RPM。
motor.SetSpeed(100.0f, 10U);
motor.SetPosition(360.0f,
                  100.0f,
                  10U,
                  10U,
                  Enum_ZDTMotor_Position_Mode::RELATIVE_TO_CURRENT);

// X：加/减速度单位 RPM/s；速度单位 RPM；位置单位度。
motor.SetSpeed(100.0f, 500U);
motor.SetPosition(360.0f,
                  100.0f,
                  500U,
                  500U,
                  Enum_ZDTMotor_Position_Mode::RELATIVE_TO_CURRENT);
```

对于 Emm，`SetPosition()` 使用 `pulses_per_revolution` 将角度换算为脉冲；`deceleration` 参数不参与 Emm 帧编码。对于 X，速度编码为 `0.1 RPM`、位置编码为 `0.1°`。负速度或负位置表示 CCW；`reverse=true` 会统一反转命令方向和反馈方向。

X 固件还支持力矩命令：

```cpp
motor.SetTorque(0.6f, 200U); // 0.6 A，电流斜率 200 mA/s
```

## 同步和回零

```cpp
motor.SetPosition(90.0f, 50.0f, 10U, 10U,
                  Enum_ZDTMotor_Position_Mode::RELATIVE_TO_CURRENT,
                  true); // 缓存，暂不执行

motor.TriggerSynchronousMotion(); // 广播 ID 0x0000，触发所有已缓存命令

motor.TriggerHoming(Enum_ZDTMotor_Homing_Mode::LIMIT_SWITCH);
motor.AbortHoming();
```

碰撞回零会主动撞击机械结构。必须先降低回零速度和电流，并准备硬件急停；人员安全不能依赖软件停止命令。

## 应答与状态

`02` 只表示命令已接受，不表示运动完成。驱动把运动状态区分为：

```text
IDLE -> WAIT_ACCEPT -> RUNNING -> DONE
                    \-> ERROR
          \-> TIMEOUT
```

- 收到 `02`：进入 `RUNNING`。
- 收到 `9F`：进入 `DONE`。
- 收到 `12`、`22`、`E2` 或 `EE`：进入 `ERROR`。
- 50 ms 内没有收到接收确认：进入 `TIMEOUT`。超时不代表电机没有执行，尤其不要立即重发相对位置命令。
- 对位置命令，轮询到 `3A` 的到位位也会进入 `DONE`。

读取一致快照：

```cpp
Struct_ZDTMotor_Feedback feedback = motor.GetFeedback();
Enum_ZDTMotor_Motion_State state = motor.GetMotionState();
```

反馈包含版本、报告固件、电压、相电流、速度、位置、位置误差、温度、电机状态、回零/保护状态、最近功能码/返回码、时间戳和校验错误计数。

## 上电验收顺序

1. 断电完成电机、电源、CAN_H、CAN_L 和共地接线，确认总线两端终端电阻。
2. 电机菜单设置 `P_Serial=CAN1_MAP`、与主控一致的 `CAN_Baud`、唯一地址和匹配的校验方式。
3. 确认电机已空载校准并处于闭环模式。
4. 在应用任务中先调用 `ReadVersion()`、`ReadOptionStatus()`，再观察 `GetFeedback()` 中的在线状态、版本、电压和温度。
5. 核对 `reported_firmware` 与 `Init()` 传入的固件一致。
6. 先使能，再用低速、低加速度、小位移测试，最后验证停止和失能。
7. 运动确认丢失时先读取 `3A`、`36` 等状态，不盲目重发相对运动。

应用层应周期调用 `Process()` 维护应答超时和在线状态。电机接线、固件设置、实际报文 ACK 和机械运动必须在硬件上按上述顺序验收。
