# Chassis 应用层说明

`Class_Chassis` 是一个薄应用层，只完成三项工作：

1. 根据 `x`、`y`、`omega` 计算四个麦克纳姆轮的目标速度；
2. 调用四个 `Class_ZDTMotor` 对象完成同步使能、速度发送、停止和失能；
3. 用 `Struct_INS_Pose` 的实时 `x/y/yaw` 和目标坐标做最简单的阻塞式位置移动。

本层不包含 PID、错误码、速度平滑、速度限幅或加速度限幅。ZDT 参数和总线配置由调用方保证正确，协议发送结果由底层 `ZDTmotor` 自行处理。

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

`SetVelocity()` 的 `x/y` 单位为 `m/s`，`omega` 单位为 `rad/s`。每个轮缘线速度结果乘以 `speed_to_motor` 后作为 RPM 传给 `Class_ZDTMotor::SetSpeed()`：

```text
speed_to_motor = gear_ratio * 60 / (2 * pi * wheel_radius_m)
```

该系数由 `Init()` 根据 `Struct_Chassis_Config` 自动计算，调用方不需要填写。

## 配置

底盘配置包含麦轮旋转项和速度单位换算参数：

```cpp
Struct_Chassis_Config chassis_config{};
chassis_config.half_length = 0.15f;
chassis_config.half_width = 0.15f;
chassis_config.wheel_radius_m = 0.05f;
chassis_config.gear_ratio = 1.0f;
```

每个车轮对应一份 ZDT 配置：

```cpp
Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
{
    motor_config[i].hfdcan = &hfdcan2;
    motor_config[i].can_id = static_cast<uint8_t>(i + 1U);
    motor_config[i].polarity = 1; // +1 正向，-1 反向
    motor_config[i].firmware = Enum_ZDTMotor_Firmware::EMM;
    motor_config[i].checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
    motor_config[i].pulses_per_revolution = 3200U;
    motor_config[i].acceleration = 10U;
}
```

四台电机使用 ZDT 同步触发时，应连接到同一条 FDCAN 总线，并使用相同校验方式和互不重复的节点地址。每台电机的 `polarity` 独立设置：`+1` 为正向，`-1` 为反向。`Init()` 会将负极性转换为 ZDTmotor 的 `reverse=true`，并自动覆盖 `speed_to_motor`。

## 接口

```cpp
Class_Chassis chassis;

chassis.Init(chassis_config, motor_config);
chassis.Enable();

// 向前运动。
chassis.SetVelocity(0.0f, 0.10f, 0.0f);
chassis.Process();

// 按指定速度运行 1.0 秒，然后同步停止。
chassis.RunForSeconds(0.0f, 0.10f, 0.0f, 1.0f);

// 阻塞移动到坐标 (0, 0.10) m、yaw = 0，位置和 yaw 到位阈值分别为 0.01 m、0.05 rad。
chassis.RunToPosition(0.0f, 0.10f, 0.01f, 0.0f, 0.05f);

// 以调用时的位置为原点，相对移动 (+0.10, +0.10) m，并保持相对 yaw 不变。
chassis.RunToRelativePosition(0.10f, 0.10f, 0.01f, 0.0f, 0.05f);

chassis.Stop();
chassis.Disable();
```

接口职责如下：

| 接口 | 行为 |
| --- | --- |
| `Init()` | 保存配置并调用四次 `Class_ZDTMotor::Init()` |
| `Enable()` | 缓存四台电机的使能命令，然后广播同步触发 |
| `SetVelocity()` | 计算四轮速度，调用 `SetSpeed()`，然后广播同步触发 |
| `RunForSeconds()` | 发送速度并用 `osDelay()` 阻塞当前任务，延时结束后同步停止 |
| `RunToPosition()` | 比较实时位置和 yaw，直接调用 `SetVelocity(error_x, error_y, error_yaw)` |
| `RunToRelativePosition()` | 用函数开始时的位姿建立原点，将相对位移换算为绝对目标后阻塞移动 |
| `Process()` | 调用四台 ZDT 电机的 `Process()`，维护应答超时和在线状态 |
| `Stop()` | 缓存四台电机的停止命令，然后广播同步触发 |
| `Disable()` | 缓存四台电机的失能命令，然后广播同步触发 |

`RunForSeconds()` 的 `duration_s` 小于等于零或不是有效正数时不会启动运动，而是立即同步停止。调用后当前任务会阻塞到指定时间结束，然后同步停止四台电机。

## 位置移动

```cpp
bool RunToPosition(float target_x_m,
                   float target_y_m,
                   float tolerance_m,
                   float target_yaw_rad = 0.0f,
                   float yaw_tolerance_rad = 0.05f);
```

实现只包含以下步骤：

```text
读取 APP_INS.GetComplementaryPosition()
error_x = target_x_m - current.x_m
error_y = target_y_m - current.y_m
error_yaw = target_yaw_rad - current.yaw_rad
if error_x^2 + error_y^2 <= tolerance_m^2 and abs(error_yaw) <= yaw_tolerance_rad:
    Stop(); return true
SetVelocity(error_x, error_y, error_yaw)
osDelay(100)
```

`error_x/error_y` 的单位是 m，按 `1 m -> 1 m/s` 直接作为 `SetVelocity()` 的 `x/y` 输入；`error_yaw_rad` 按 `1 rad -> 1 rad/s` 直接作为 `omega` 输入。`SetVelocity()` 再用轮径和减速比完成线速度到 RPM 换算。没有 PID、坐标旋转、速度斜坡或加减速控制。

### 相对位置移动

```cpp
bool RunToRelativePosition(float relative_x_m,
                           float relative_y_m,
                           float tolerance_m,
                           float relative_yaw_rad = 0.0f,
                           float yaw_tolerance_rad = 0.05f);
```

函数进入时只读取一次 `APP_INS.GetComplementaryPosition()`，并保存为本次移动的原点：

```text
origin = APP_INS.GetComplementaryPosition()
target_x_m = origin.x_m + relative_x_m
target_y_m = origin.y_m + relative_y_m
target_yaw_rad = origin.yaw_rad + relative_yaw_rad
```

随后按照 `RunToPosition()` 的流程循环比较实时位姿和目标位姿。原点在循环中不会更新，因此 `relative_x_m`、`relative_y_m` 和 `relative_yaw_rad` 均相对于函数开始时的车体位姿。

## 当前 Control_Task

`User_File/Task/Control_Task.cpp` 使用 FDCAN2 上地址为 `1~4` 的四台 ZDT EMM 电机：

- 上电等待 2 s；
- 初始化并同步使能四台电机；
- 1、3 号电机使用正极性，2、4 号电机使用反极性；
- 等待 200 ms；
- 调用 `RunToPosition(0.3, 0.6, 0.01, 0.0, 0.05)` 和 `RunToPosition(0, 0, 0.01, 0.0, 0.05)`；
- 循环等待使用 `osDelay(100)`；
- 任务进入永久等待。

ZDT 固件、校验、反馈和命令格式见 [`../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md`](../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md)。
