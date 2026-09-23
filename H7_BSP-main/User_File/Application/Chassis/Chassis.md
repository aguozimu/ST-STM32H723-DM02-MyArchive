# Chassis 应用层说明

`Class_Chassis` 是一个薄应用层，只完成三项工作：

1. 根据 `x`、`y`、`omega` 计算四个麦克纳姆轮的目标速度；
2. 调用四个 `Class_ZDTMotor` 对象完成同步使能、速度发送、停止和失能；
3. 用 `INS` 的位姿做最简距离闭环 `RunToRelativePose()`。

本层不建立故障状态机、不做打滑或电机离线判定，也不做速度前馈和轨迹规划。公开接口只做最必要的参数判断，ZDT 参数和总线配置由调用方保证正确，协议发送结果由底层 `ZDTmotor` 自行处理。

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
chassis_config.half_length = 0.073f;        // 中心到前/后轮轴，m
chassis_config.half_width = 0.146f;         // 中心到左/右轮接触中心，m
chassis_config.wheel_radius_m = 0.0375f;    // 麦轮有效直径 75 mm
chassis_config.gear_ratio = 1.0f;           // 电机转数 / 麦轮转数，直驱填 1
chassis_config.max_linear_speed_m_s = 0.30f;   // 距离闭环最大平移速度
chassis_config.max_angular_speed_rad_s = 1.0f; // 距离闭环最大角速度
```

`speed_to_motor` 不再手填，`Init()` 会按 `gear_ratio * 60 / (2 * pi * wheel_radius_m)` 算出，把麦轮线速度 m/s 换算成电机转速 RPM。

每个车轮对应一份 ZDT 配置，数组顺序固定为左前、右前、左后、右后：

```cpp
Struct_Chassis_Motor_Config motor_config[CHASSIS_WHEEL_COUNT]{};
for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
{
    motor_config[i].hfdcan = &hfdcan2;
    motor_config[i].can_id = static_cast<uint8_t>(i + 1U);
    motor_config[i].wheel = static_cast<Enum_Chassis_Wheel>(i); // 记录用
    motor_config[i].roller_orientation = Chassis_Roller_ForwardSlash; // 记录用
    motor_config[i].polarity = 1; // +1 正向，-1 反向
    motor_config[i].firmware = Enum_ZDTMotor_Firmware::EMM;
    motor_config[i].checksum = Enum_ZDTMotor_Checksum::FIXED_6B;
    motor_config[i].pulses_per_revolution = 3200U;
    motor_config[i].acceleration = 10U;
}
```

`wheel` 和 `roller_orientation` 只用于配置记录和人工核对，运动学公式按标准 X 型麦轮构型写死；`speed_to_motor` 由 `Init()` 覆盖，不用填。

四台电机使用 ZDT 同步触发时，应连接到同一条 FDCAN 总线，并使用相同校验方式和互不重复的节点地址。每台电机的 `polarity` 独立设置：`+1` 为正向，`-1` 为反向。`Init()` 会将负极性转换为 ZDTmotor 的 `reverse=true`，命令方向和位置/速度反馈方向会同时翻转，因此 INS 和正运动学里不要再按左右轮改符号。

## 接口

```cpp
Class_Chassis chassis;

chassis.Init(chassis_config, motor_config);
chassis.Enable();

// 向前运动。
chassis.SetVelocity(0.0f, 50.0f, 0.0f);
chassis.Process();

// 按指定速度运行 1.0 秒，然后同步停止。
chassis.RunForSeconds(0.0f, 50.0f, 0.0f, 1.0f);

chassis.Stop();
chassis.Disable();
```

接口职责如下：

| 接口 | 行为 |
| --- | --- |
| `Init()` | 保存配置并调用四次 `Class_ZDTMotor::Init()` |
| `Enable()` | 缓存四台电机的使能命令，然后广播同步触发 |
| `SetVelocity()` | 计算四轮速度，调用 `SetSpeed()`，然后广播同步触发 |
| `RunForSeconds()` | 发送速度并设置毫秒倒计时，立即返回；倒计时结束后同步停止 |
| `TIM_1ms_RunForSeconds_PeriodElapsedCallback()` | 由 `TIM_1ms_Task` 每 1 ms 调用，维护定时运行倒计时 |
| `Process()` | 调用四台 ZDT 电机的 `Process()`，维护应答超时和在线状态 |
| `Stop()` | 缓存四台电机的停止命令，然后广播同步触发 |
| `Disable()` | 缓存四台电机的失能命令，然后广播同步触发 |
| `ConfigureOdometryFeedback()` | 为四台电机配置 `0x36` 实时位置的周期返回，周期写 0 停止 |
| `StopOdometryFeedback()` | 等价于 `ConfigureOdometryFeedback(0)` |
| `GetWheelFeedback()` | 复制四轮反馈快照，位置和转速已应用极性，供 INS 使用 |
| `RunToRelativePose()` | 相对当前位姿的距离闭环，阻塞到到位、超时或参数非法 |
| `SetWheelPolarity()` | 停车状态下修改单轮极性，只改 ZDTmotor 的方向标志 |

`RunForSeconds()` 的 `duration_s` 小于等于零或不是有效正数时不会启动运动，而是立即同步停止。时长小于 1 ms 时按 1 ms 运行，不能整除到毫秒的时长向上取整；直接调用 `SetVelocity()`、`Stop()` 或 `Disable()` 会取消尚未结束的定时运行倒计时。

## 距离闭环

```cpp
bool RunToRelativePose(float x, float y, float omega, float distance_m);
```

`x`、`y` 是机体系方向，`+x` 向右、`+y` 向前，只有方向有意义，模长既不表示距离也不表示速度；`distance_m` 是要走的距离；`omega` 是相对调用瞬间朝向的目标转角，逆时针为正。

| 调用 | 含义 |
| --- | --- |
| `RunToRelativePose(0, 1, 0, 1.0f)` | 沿当前车头走 1 m |
| `RunToRelativePose(1, 0, 0, 0.5f)` | 沿当前右方横移 0.5 m |
| `RunToRelativePose(1, 1, 0, 1.0f)` | 沿右前 45 度走 1 m |
| `RunToRelativePose(0, 0, 1.5707963f, 0)` | 原地逆时针转 90 度 |

实现要点：

- 目标点由调用瞬间的位姿算出，`dx_body / dy_body` 转到世界系后叠加到起点；
- 世界系误差换算到机体系后用 P 控制，`kp_xy = 1.2`、`kp_yaw = 2.0`，并按 `max_linear_speed_m_s`、`max_angular_speed_rad_s` 限幅，再交给麦轮逆运动学；
- 到位判据为 `|ex| < 0.02 m`、`|ey| < 0.02 m`、`|eyaw| < 0.05 rad` 同时成立，到位后调用 `Stop()` 并返回 `true`；
- 累计 60 s 仍未到位就 `Stop()` 并返回 `false`，只用于防止死循环；
- `distance_m == 0 && omega == 0` 是空操作，直接返回 `true`；`distance_m > 0` 而 `x == y == 0`、出现 NaN/Inf、`distance_m < 0`、INS 未就绪都返回 `false`；
- 内部使用 `osDelay()`，只能在 `Control_Task` 里调用，阻塞期间 InsTask 照常更新位姿。

## 当前 Control_Task

`User_File/Task/Control_Task.cpp` 使用 FDCAN2 上地址为 `1~4` 的四台 ZDT EMM 电机：

- 上电等待 2 s；
- 初始化底盘和 INS，机械参数为 `half_length = 0.073 m`、`half_width = 0.146 m`、麦轮有效半径 `0.0375 m`、减速比 `1:1`；
- 打开四轮 `0x36` 周期位置反馈（10 ms）并同步使能四台电机；
- 等待 400 ms，让 InsTask 建立位姿基准；
- 依次执行 `RunToRelativePose(0, 1, 0, 1.0f)`、`RunToRelativePose(1, 0, 0, 0.5f)`、`RunToRelativePose(0, 0, 1.5707963f, 0)`；
- 任务进入永久等待。

极性和滚子方向沿用台架初值（1、3 正极性，2、4 反极性），必须按实车测量结果修改；标定方法见 `ref/` 下的设计说明第 9、11 节。

ZDT 固件、校验、反馈和命令格式见 [`../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md`](../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md)。
