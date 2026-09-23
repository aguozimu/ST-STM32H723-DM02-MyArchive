# INS 应用层说明

`Class_INS` 只做一件事：把四轮位置反馈和 BMI088 的 z 轴角速度融合成机体系位姿 `x / y / yaw`。第一版不做 EKF/ESKF、不做打滑检测、不做电机在线判定，也不做时间同步和协方差传播。

## 数据流

```text
四轮 0x36 位置反馈 --> 麦轮正运动学 dx_body / dy_body / dyaw_wheel --+
                                                                     +--> 互补融合 --> x, y, yaw
BMI088 gyro_z (rad/s) --> dyaw_gyro = gyro_z * dt -------------------+
```

输入来源：

| 来源 | 接口 | 含义 |
| --- | --- | --- |
| 四轮位置 | `APP_Chassis.GetWheelFeedback().position_deg[i]` | 已应用极性，单位度 |
| 陀螺仪 | `BSP_BMI088.Get_Gyro_Body().Data[2]` | VQF 修正后的机体系 z 轴角速度，rad/s |
| 时间 | `SYS_Timestamp.Get_Now_Microsecond()` | 64 位微秒时间戳 |

## 坐标系与单位

沿用 `Class_Chassis::SetVelocity()` 的定义：机体系 `+x` 向右、`+y` 向前、`+z` 向上，`+yaw` 逆时针为正；长度 m，角度 rad，角速度 rad/s。复位瞬间的车体坐标系就是世界系，之后不再改变，复位时 `x = y = yaw = 0`。

## 公式

轮子位移只做单位换算和 ±180 度回绕处理：

```text
d_deg = WrapAngleDeg(position_deg_now - position_deg_last)   // 折算到 [-180, 180)
ds    = d_deg * pi / 180 / gear_ratio * wheel_radius_m        // 单位 m
```

麦轮正运动学与 `SetVelocity()` 的逆运动学严格对称：

```text
dx_body    = ( ds[LF] - ds[RF] - ds[LR] + ds[RR]) / 4
dy_body    = ( ds[LF] + ds[RF] + ds[LR] + ds[RR]) / 4
dyaw_wheel = (-ds[LF] + ds[RF] - ds[LR] + ds[RR]) / (4 * (half_length + half_width))
```

航向互补融合与中点航向积分：

```text
dyaw_gyro = gyro_z * dt
dyaw      = 0.98 * dyaw_gyro + 0.02 * dyaw_wheel
yaw_mid   = yaw + 0.5 * dyaw
x        += cos(yaw_mid) * dx_body - sin(yaw_mid) * dy_body
y        += sin(yaw_mid) * dx_body + cos(yaw_mid) * dy_body
yaw       = WrapAngle(yaw + dyaw)
```

`GYRO_WEIGHT` 越大越信任陀螺仪。原地旋转时如果轮式角度和陀螺仪角度差得多，说明轮子打滑或极性配错，先按设计说明第 9 节检查极性。

## 接口

| 接口 | 行为 |
| --- | --- |
| `Init(config)` | 用底盘配置的轮径、减速比和半长/半宽初始化，并复位位姿 |
| `ResetPose()` | 世界系原点回到当前车体，只清导航原点，不清电机内部计数 |
| `Update()` | 融合一帧四轮位置和陀螺仪数据；由 InsTask 以 10 ms 周期调用 |
| `GetState()` | 取最近一次位姿快照 `x_m / y_m / yaw_rad / valid` |
| `UpdateVisionPose()` / `GetVisionPose()` | 视觉绝对定位预留接口，第一版只缓存最新一帧，不参与融合 |

## 约束与实现细节

1. `Update()` 只做融合和积分，不判有效、不判超时、不判打滑；
2. 位置反馈按度处理，差值先折算到 `[-180, 180)`，单圈和多圈编码都能用；
3. 各轮的位置基准按轮建立：某轮还没收到过位置反馈时先建立基准，该周期不计入位移，避免把电机上电时的绝对位置当成一次大位移；
4. `dt` 由微秒时间戳算出，`dt <= 0` 的周期直接跳过，不积分；
5. `gear_ratio` 是除数，配置非法（小于等于 0）时按直驱 1 处理，避免出现除零；
6. 全局共享同一个实例 `APP_INS`，与 `APP_Chassis` 配套使用，`Init()` 必须传底盘同一份配置。

## 相关代码

- 应用层实现：[`INS.h`](INS.h)、[`INS.cpp`](INS.cpp)
- 速度解算与距离闭环：[`../Chassis/Chassis.md`](../Chassis/Chassis.md)
- 电机协议：[`../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md`](../../Device/Peripheral/Motor/ZDTmotor/zdtmotor.md)
