# RMMotor

RoboMaster 电机驱动模块（M2006 / M3508 / GM6020）：CAN 收发、反馈解码与 Motor 接口 / RoboMaster motor driver Module (M2006 / M3508 / GM6020) with CAN transfer, feedback decoding and the Motor interface

## 1. 模块作用 / Purpose

RMMotor 实现 `Motor` 接口，驱动 M2006、M3508 与 GM6020 电机。构造时按型号和 `feedback_id` 确定控制帧 ID 和本电机在控制帧中的槽位，并注册该反馈 ID 的标准帧回调。接收队列深度为 1，只保留最新一帧反馈。

| 型号 | `feedback_id` | 控制帧 ID |
| --- | --- | --- |
| M2006 / M3508 | `0x201`–`0x204` | `0x200` |
| M2006 / M3508 | `0x205`–`0x208` | `0x1FF` |
| GM6020 | `0x205`–`0x208` | `0x1FE` |
| GM6020 | `0x209`–`0x20B` | `0x2FE` |

同一条 CAN 总线（按 `LibXR::CAN` 对象区分）、同一控制帧 ID 的电机共享一个 8 字节发送帧，每个电机占 2 字节。组内所有已构造的电机在本轮都写入命令后才发送这一帧，因此同组的每个电机每个周期调用一次 `Control()`（或 `Relax()` / `Disable()`）。

`Update()` 解码反馈：`position` 为编码器值 / 8192 × 2π（单圈，rad），`velocity` 单位为 rpm，`omega` 单位为 rad/s，`torque` 由反馈电流按型号力矩常数换算，`temp` 单位为 ℃；收到反馈后 `state = 1`，`abs_angle` 取 `position`。连续超过 255 次调用未收到反馈时返回 `ErrorCode::NO_RESPONSE`，否则返回 `OK`。

`Control()` 处理两种模式，其余模式被忽略：

| `MotorCmd::mode` | 行为 |
| --- | --- |
| `MODE_TORQUE` | 输出轴力矩 `torque` 除以 `reduction_ratio`，按型号力矩常数和最大电流换算为控制量 |
| `MODE_CURRENT` | `velocity` 字段给出归一化电流 [-1, 1]，乘以满量程控制量 |

`Disable()` 与 `Relax()` 下发 0 电流；`Enable()`、`ClearError()` 与 `SaveZeroPoint()` 为空实现。`reverse = true` 时反馈的位置和转速取反，输出也取反（力矩换算基于原始电流）。反馈温度超过 75 ℃ 时输出置 0，并输出 `XR_LOG_WARN`。额外的公共接口有 `TorqueControl(torque, reduction_ratio)` 与 `GetOmega()`。

型号参数：

| 型号 | 力矩常数 (N·m/A) | 最大电流 (A) | 满量程控制量 |
| --- | --- | --- | --- |
| `MOTOR_M2006` | 0.005 | 10 | 10000 |
| `MOTOR_M3508` | 0.0156224 | 20 | 16384 |
| `MOTOR_GM6020` | 0.741 | 3 | 16384 |

RMMotor implements the `Motor` interface and drives M2006, M3508 and GM6020 motors. At construction it determines the control frame ID and the slot of the motor in the control frame from the model and `feedback_id`, and registers a standard-frame callback for that feedback ID. The receive queue has depth 1 and keeps only the latest feedback frame.

| Model | `feedback_id` | Control frame ID |
| --- | --- | --- |
| M2006 / M3508 | `0x201`–`0x204` | `0x200` |
| M2006 / M3508 | `0x205`–`0x208` | `0x1FF` |
| GM6020 | `0x205`–`0x208` | `0x1FE` |
| GM6020 | `0x209`–`0x20B` | `0x2FE` |

Motors on the same CAN bus (distinguished by the `LibXR::CAN` object) with the same control frame ID share one 8-byte transmit frame, with 2 bytes per motor. The frame is sent once every constructed motor of the group has written a command in the current round, so each motor of a group calls `Control()` (or `Relax()` / `Disable()`) once per cycle.

`Update()` decodes the feedback: `position` is the encoder value / 8192 × 2π (single turn, rad), `velocity` is in rpm, `omega` in rad/s, `torque` is converted from the feedback current with the model torque constant, and `temp` is in ℃. After feedback arrives `state = 1`, and `abs_angle` takes `position`. After more than 255 consecutive calls without feedback it returns `ErrorCode::NO_RESPONSE`, otherwise `OK`.

`Control()` handles two modes and ignores the others:

| `MotorCmd::mode` | Behavior |
| --- | --- |
| `MODE_TORQUE` | The output-shaft torque `torque` is divided by `reduction_ratio` and converted to the control value with the model torque constant and maximum current |
| `MODE_CURRENT` | The `velocity` field carries the normalized current [-1, 1], multiplied by the full-scale control value |

`Disable()` and `Relax()` send 0 current; `Enable()`, `ClearError()` and `SaveZeroPoint()` are empty. With `reverse = true` the feedback position and speed are negated and the output is negated as well (the torque conversion uses the raw current). When the feedback temperature exceeds 75 ℃ the output is set to 0 and `XR_LOG_WARN` is emitted. Additional public interfaces are `TorqueControl(torque, reduction_ratio)` and `GetOmega()`.

Model parameters:

| Model | Torque constant (N·m/A) | Maximum current (A) | Full-scale control value |
| --- | --- | --- | --- |
| `MOTOR_M2006` | 0.005 | 10 | 10000 |
| `MOTOR_M3508` | 0.0156224 | 20 | 16384 |
| `MOTOR_GM6020` | 0.741 | 3 | 16384 |

## 2. 构造接口 / Constructor

```cpp
RMMotor(LibXR::CAN& can_bus,
        const Param& param = {.model = RMMotor::Model::MOTOR_M3508,
                              .reverse = false,
                              .feedback_id = 0x201});
```

依赖：

- `can_bus`：`LibXR::CAN`，电机所在的 CAN 总线，取自 BSP 的硬件注册（`XR_REGISTER`）。

配置参数（`Param`）：

- `model`：电机型号，`RMMotor::Model::MOTOR_M2006`、`MOTOR_M3508`、`MOTOR_GM6020` 或 `MOTOR_NONE`，默认 `MOTOR_M3508`。
- `reverse`：是否反向，默认 `false`。
- `feedback_id`：电机反馈帧 ID（见第 1 节），默认 `0x201`。

Dependencies:

- `can_bus`: the `LibXR::CAN` the motor is attached to, taken from the BSP's Registration (`XR_REGISTER`).

Configuration parameters (`Param`):

- `model`: motor model, `RMMotor::Model::MOTOR_M2006`, `MOTOR_M3508`, `MOTOR_GM6020` or `MOTOR_NONE`, default `MOTOR_M3508`.
- `reverse`: whether to reverse the direction, default `false`.
- `feedback_id`: motor feedback frame ID (see section 1), default `0x201`.

## 3. Topic

无 / None

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/RMMotor` 写入的实例，`can_bus` 填写为 BSP 中注册的 CAN 名称，`model` 与 `feedback_id` 按电机填写：

An instance written by `xrobot instance add QDU-Robomaster/RMMotor`, with `can_bus` set to a CAN name registered by the BSP, and `model` and `feedback_id` set according to the motor:

```yaml
modules:
  - module: QDU-Robomaster/RMMotor
    id: motor_pit
    args:
      - can_bus: can1
      - param:
          model: RMMotor::Model::MOTOR_GM6020
          reverse: false
          feedback_id: 0x209
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/Motor`：本模块实现的电机接口。
- LibXR。

硬件：M2006、M3508 或 GM6020 电机，经 `LibXR::CAN` 连接，CAN 对象通过 `XR_REGISTER` 注册。

Dependencies:

- `QDU-Robomaster/Motor`: the motor interface implemented by this Module.
- LibXR.

Hardware: M2006, M3508 or GM6020 motors connected through `LibXR::CAN`, with the CAN object registered with `XR_REGISTER`.
