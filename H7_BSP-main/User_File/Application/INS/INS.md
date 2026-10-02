# INS 应用层说明

`Class_INS` 将四轮编码器里程计和 BMI088 陀螺仪 Yaw 组合为底盘局部二维位姿：

- X/Y 只由四轮编码器和麦克纳姆轮正运动学提供；
- BMI088 只使用机体系 Z 轴角速度参与 Yaw 互补融合；
- 不使用加速度计进行 X/Y 二次积分；
- 不包含静止零速修正、打滑检测、反馈超时或 EKF/ESKF。

## 数据流

```text
四轮 0x36 位置反馈
        |
        v
麦轮正运动学 ---> dx_body / dy_body / dyaw_wheel ---+
                                                     |
                                                     +--> 互补滤波 --> x, y, yaw
BMI088 gyro_z (rad/s) --> dyaw_gyro = gyro_z * dt --+
```

## 输入来源

| 来源 | 接口 | 含义 |
| --- | --- | --- |
| 四轮位置 | `APP_Chassis.GetMotorFeedback()` | `ConfigurePeriodicFeedback()` 配置的 0x36 位置反馈，单位 deg |
| 陀螺仪 | `BSP_BMI088.Get_Gyro_Body().Data[2]` | VQF 修正后的机体系 Z 轴角速度，单位 rad/s |
| 时间 | `SYS_Timestamp.Get_Now_Microsecond()` | 64 位微秒时间戳 |
| 配置 | `APP_Chassis.ConfigurePeriodicFeedback()` | 默认按 `INS_UPDATE_PERIOD_MS = 10 ms` 配置 |

## 坐标系与单位

沿用 `Class_Chassis::SetVelocity()` 的方向定义：

- 机体系 `+x` 向右；
- 机体系 `+y` 向前；
- 机体系 `+z` 向上；
- `yaw` 逆时针为正；
- 位置单位 m，角度单位 rad，角速度单位 rad/s。

复位瞬间的车体系就是世界系，之后世界系不再改变。复位时：

```text
x = y = yaw = 0
```

## 函数 1：轮式里程计

每个轮子的位置差先折算到 `[-180, 180)` 度，再使用轮径和减速比换算为轮缘位移：

```text
d_deg = WrapAngleDeg(position_deg_now - position_deg_last)
ds    = d_deg * pi / 180 / gear_ratio * wheel_radius_m
```

四轮都收到新反馈后执行麦轮正运动学：

```text
dx_body    = ( ds[LF] - ds[RF] - ds[LR] + ds[RR]) / 4
dy_body    = ( ds[LF] + ds[RF] + ds[LR] + ds[RR]) / 4
dyaw_wheel = (-ds[LF] + ds[RF] - ds[LR] + ds[RR]) / (4 * (half_length + half_width))
```

处理策略：

1. 每个轮子第一次收到反馈时只建立位置基准，不产生位移；
2. 某个轮子提前到达时先累计 `wheel_pending_m`；
3. 四个轮子都有新反馈后才统一计算一组正运动学结果；
4. 使用周期中点航向积分，减少边走边转时的离散误差。

```text
yaw_mid = wrap(yaw + dyaw / 2)
x += cos(yaw_mid) * dx_body - sin(yaw_mid) * dy_body
y += sin(yaw_mid) * dx_body + cos(yaw_mid) * dy_body
yaw = wrap(yaw + dyaw)
```

## 函数 2：陀螺仪 Yaw

函数 2 每个 10 ms 周期执行一次：

```text
now = SYS_Timestamp.Get_Now_Microsecond()
dt  = (now - last_gyro_timestamp_us) / 1e6
dyaw_gyro = gyro_z * dt
yaw_gyro = wrap(yaw_gyro + dyaw_gyro)
```

其中：

- `gyro_z = BSP_BMI088.Get_Gyro_Body().Data[2]`；
- `dt <= 0`、`dt > 0.1 s` 或角速度不是有限值时，本周期不积分；
- 函数 2 的 X/Y 仍沿用同一周期的轮式平移，只使用陀螺仪航向进行旋转；
- 加速度计不参与 X/Y 计算。

## 函数 3：互补滤波

函数 3 根据序号判断是否收到新的轮式增量和陀螺仪增量：

```text
dyaw = gyro_weight * dyaw_gyro + (1 - gyro_weight) * dyaw_wheel
```

- `gyro_weight` 越大，越信任陀螺仪航向；
- `gyro_weight` 越小，越信任轮式航向；
- 只有一侧有新数据时，退化为使用该侧航向增量；
- 平移增量始终取函数 1 的轮式正运动学结果；
- 融合位姿使用同一套中点航向积分。

## 接口职责

| 接口 | 行为 |
| --- | --- |
| `Init(config)` | 保存轮径、减速比、半长/半宽和互补权重，并复位位姿 |
| `Reset()` | 清零三个位姿和所有差分基准，不清电机内部计数 |
| `UpdateWheelPosition()` | 处理四轮位置反馈并执行麦克纳姆轮正运动学 |
| `UpdateGyroPosition()` | 积分 BMI088 Z 轴角速度并维护陀螺仪航向位姿 |
| `UpdateComplementaryPosition()` | 融合轮式与陀螺仪 Yaw，输出最终位姿 |
| `GetWheelPosition()` | 读取轮式正运动学位姿 |
| `GetGyroPosition()` | 读取陀螺仪航向位姿 |
| `GetComplementaryPosition()` | 读取互补滤波位姿 |
| `GetState()` | 一次读取三个位姿快照 |
| `StartCalibration()` | 记录当前三组世界系位姿，作为运行时误差基准 |
| `EndCalibration()` | 用当前位姿与记录位姿的差值修正世界系原点 |

## 世界系校准

世界系来自复位瞬间的车体系，长时间运行后原点会累积误差。两个函数用一次已知的
闭环移动（外部用 `SetVelocity()` 驱动）把世界系原点重新对齐：

1. `StartCalibration()`：记录当前函数 1/2/3 的三组世界系位姿快照；
2. 外部调用 `SetVelocity()` 完成校准移动；
3. `EndCalibration()`：读取当前位姿与记录位姿的差值（本次运行产生的误差）：

```text
error = current - recorded
pose  = pose - error
```

三个位姿同处一个世界系，统一扣除函数 3 互补滤波位姿的误差，等价于把世界系原点
从校准起点平移到校准终点。校准只修正静态原点误差，不修正轮径、减速比等比例误差。

典型用法：

```cpp
if (APP_INS.StartCalibration())
{
    /* 外部用 SetVelocity() 完成校准移动 */
    APP_INS.EndCalibration();
}
```

三组位姿和校准数据都由 `Class_INS` 内部的互斥锁保护，周期函数与外部任务可以并发调用。

## 初始化

应在 `Class_Chassis::Init()` 之后调用：

```cpp
Struct_INS_Config ins_config{};
ins_config.half_length_m = 0.15f;
ins_config.half_width_m = 0.15f;
ins_config.wheel_radius_m = 0.05f;
ins_config.gear_ratio = 1.0f;
ins_config.gyro_weight = 0.98f;
APP_INS.Init(ins_config);
```

`APP_INS.Init()` 会调用：

```cpp
APP_Chassis.ConfigurePeriodicFeedback(INS_UPDATE_PERIOD_MS);
```

## 定时器注册

三个函数按 `TIM_1ms_Task.cpp` 的 `pulse()` 规则注册到 10 ms 周期；同一组中的 WS2812 刷新不影响执行顺序：

```cpp
pulse(10, 4,
      INS_TIM_10ms_UpdateWheelPosition_PeriodElapsedCallback,
      INS_TIM_10ms_UpdateGyroPosition_PeriodElapsedCallback,
      INS_TIM_10ms_UpdateComplementaryPosition_PeriodElapsedCallback,
      BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback);
```

注册顺序保证函数 3 使用同一节拍内已经更新的轮式位姿和陀螺仪航向。

## UART7 遥测

`TransportTask.cpp` 使用 UART7 发送 justfloat 帧，当前 VOFA+ 0 基通道映射为：

| 通道 | 数据 |
| --- | --- |
| 0~2 | `Debug_IMU_Data.Original_Accel_X/Y/Z_m_s2` |
| 3~6 | 四轮转速 `motor_speed_rpm[]` |
| 7~10 | 四轮位置 `motor_position_deg[]` |
| 11~13 | 函数 1 轮式位姿 `x_m / y_m / yaw_rad` |
| 14~16 | 函数 2 陀螺仪航向位姿 `x_m / y_m / yaw_rad` |
| 17~19 | 函数 3 互补滤波位姿 `x_m / y_m / yaw_rad` |

## 约束

1. 本层只做里程计和 Yaw 融合，不判断反馈有效性、超时或打滑；
2. 位置反馈按度处理，差分前必须进行角度包装；
3. 各轮位置基准按轮建立，避免把上电绝对位置当成一次大位移；
4. 世界系来自复位瞬间的机体系，没有绝对位置或绝对航向观测时长期会漂移；
5. `gear_ratio` 非正时按直驱 1 处理；
6. 所有任务共享全局实例 `APP_INS`；
7. 世界系校准只扣除一次运行的累计误差，不做比例标定。
