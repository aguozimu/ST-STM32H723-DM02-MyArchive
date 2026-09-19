# Chassis 应用层说明

`Class_Chassis` 是一个薄应用层，只完成两项工作：

1. 根据 `x`、`y`、`omega` 计算四个麦克纳姆轮的目标速度；
2. 调用四个 `Class_ZDTMotor` 对象完成同步使能、速度发送、停止和失能。

本层不包含参数校验、错误码、状态机、速度平滑、速度限幅或加速度限幅。ZDT 参数和总线配置由调用方保证正确，协议发送结果由底层 `ZDTmotor` 自行处理。

## 车轮顺序

所有四元素数组固定使用以下顺序：

| 下标 | 枚举 | 车轮 |
| --- | --- | --- |
| 0 | `Chassis_Wheel_LeftFront` | 左前 |
| 1 | `Chassis_Wheel_RightFront` | 右前 |
| 2 | `Chassis_Wheel_LeftRear` | 左后 |
| 3 | `Chassis_Wheel_RightRear` | 右后 |

## 速度解算

坐标方向定义：

- `x > 0`：向右；
- `y > 0`：向前；
- `omega > 0`：逆时针旋转。

设 `R = half_length + half_width`，四轮速度为：

```text
left_front  = y + x - R * omega
right_front = y - x + R * omega
left_rear   = y - x - R * omega
right_rear  = y + x + R * omega
```

每个结果乘以对应的 `speed_to_motor` 后作为 RPM 传给 `Class_ZDTMotor::SetSpeed()`。如果 `x`、`y` 已经使用 RPM 且 `omega` 为零，可以将 `speed_to_motor` 设为 `1.0f`。

## 配置

底盘配置只保留麦轮旋转项所需的几何参数：

```cpp
Struct_Chassis_Config chassis_config{};
chassis_config.half_length = 0.15f;
chassis_config.half_width = 0.15f;
```

每个车轮对应一份 ZDT 配置：

```cpp
Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
{
    motor_config[i].hfdcan = &hfdcan2;
    motor_config[i].can_id = static_cast<uint8_t>(i + 1U);
    motor_config[i].polarity = 1; // +1 正向，-1 反向
    motor_config[i].speed_to_motor = 1.0f;
    motor_config[i].firmware = Enum_ZDTMotor_Firmware::EMM;
    motor_config[i].checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
    motor_config[i].pulses_per_revolution = 3200U;
    motor_config[i].acceleration = 10U;
}
```

四台电机使用 ZDT 同步触发时，应连接到同一条 FDCAN 总线，并使用相同校验方式和互不重复的节点地址。每台电机的 `polarity` 独立设置：`+1` 为正向，`-1` 为反向。`Init()` 会将负极性转换为 ZDTmotor 的 `reverse=true`。

## 接口

```cpp
Class_Chassis chassis;

chassis.Init(chassis_config, motor_config);
chassis.Enable();

// 向前运动。
chassis.SetVelocity(0.0f, 50.0f, 0.0f);
chassis.Process();

chassis.Stop();
chassis.Disable();
```

接口职责如下：

| 接口 | 行为 |
| --- | --- |
| `Init()` | 保存配置并调用四次 `Class_ZDTMotor::Init()` |
| `Enable()` | 缓存四台电机的使能命令，然后广播同步触发 |
| `SetVelocity()` | 计算四轮速度，调用 `SetSpeed()`，然后广播同步触发 |
| `Process()` | 调用四台 ZDT 电机的 `Process()`，维护应答超时和在线状态 |
| `Stop()` | 缓存四台电机的停止命令，然后广播同步触发 |
| `Disable()` | 缓存四台电机的失能命令，然后广播同步触发 |

## 当前 Control_Task

`User_File/Task/Control_Task.cpp` 使用 FDCAN2 上地址为 `1~4` 的四台 ZDT EMM 电机：

- 上电等待 2 s；
- 初始化并同步使能四台电机；
- 1、3 号电机使用正极性，2、4 号电机使用反极性；
- 等待 200 ms；
- 调用一次 `SetVelocity(0, 50, 0)`，以 50 RPM 向前运行；
- 以 10 ms 周期调用 `Process()`，1 s 后结束运行；
- 同步停止并失能四台电机；
- 任务进入永久等待。

ZDT 固件、校验、反馈和命令格式见 [`../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md`](../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md)。
