# RMMotor

RoboMaster 电机驱动模块（M2006 / M3508 / GM6020）。实现 CAN 收发、反馈解码和 `Motor` 抽象接口。

- 构造时根据型号与 `feedback_id` 推导控制帧 ID 和本电机在控制帧中的槽位，并注册该反馈 ID 的
  标准帧回调。接收队列深度为 1，只保留最新一帧。

  | 型号 | `feedback_id` | 控制帧 ID |
  | --- | --- | --- |
  | M2006 / M3508 | `0x201`–`0x204` | `0x200` |
  | M2006 / M3508 | `0x205`–`0x208` | `0x1FF` |
  | GM6020 | `0x205`–`0x208` | `0x1FE` |
  | GM6020 | `0x209`–`0x20B` | `0x2FE` |

- 拼包发送：同一条 CAN 总线（按 `LibXR::CAN` 对象区分）、同一控制帧 ID 的电机共享一个 8 字节
  发送帧，每个电机占 2 字节。只有组内所有已构造的电机在本轮都写入过命令后，才发送这一帧；
  因此同组的每个电机每个周期都要调用一次 `Control()`（或 `Relax()` / `Disable()`）。
- `Update()` 解码反馈：`position` = 编码器值 / 8192 × 2π（单圈，rad）、`velocity`（rpm）、
  `omega`（rad/s）、`torque`（由反馈电流按型号力矩常数换算）、`temp`（℃），收到反馈后
  `state = 1`；`abs_angle` 取 `position`。连续超过 255 次调用未收到反馈时返回
  `ErrorCode::NO_RESPONSE`，否则返回 `OK`。
- `Control()` 只处理两种模式，其余模式忽略：

  | `MotorCmd::mode` | 行为 |
  | --- | --- |
  | `MODE_TORQUE` | 输出轴力矩 `torque` 除以 `reduction_ratio`，按型号力矩常数和最大电流换算为控制量 |
  | `MODE_CURRENT` | 从 **`velocity` 字段** 读取归一化电流 [-1, 1]，乘以满量程控制量 |

- `Enable()` 为空实现；`Disable()` 与 `Relax()` 下发 0 电流；`ClearError()`、`SaveZeroPoint()`
  为空实现。
- `reverse = true` 时反馈的位置、转速取反（扭矩换算基于原始电流），输出也取反。
- 温度保护：反馈温度超过 75 ℃ 时输出置 0 并输出 `XR_LOG_WARN`。
- 额外公共接口：`TorqueControl(torque, reduction_ratio)`、`GetOmega()`。

型号参数（代码常量）：

| 型号 | 力矩常数 (N·m/A) | 最大电流 (A) | 满量程控制量 |
| --- | --- | --- | --- |
| `MOTOR_M2006` | 0.005 | 10 | 10000 |
| `MOTOR_M3508` | 0.0156224 | 20 | 16384 |
| `MOTOR_GM6020` | 0.741 | 3 | 16384 |

## 依赖

- `QDU-Robomaster/Motor`：本模块实现的电机抽象接口（库）。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
RMMotor(LibXR::CAN& can_bus,
        const Param& param = {
            .model = RMMotor::Model::MOTOR_M3508, .reverse = false, .feedback_id = 0x201});
```

依赖：

- `can_bus`：`LibXR::CAN`，电机所在的 CAN 总线。

配置（`Param`）：

- `model`：`RMMotor::Model::MOTOR_M2006` / `MOTOR_M3508` / `MOTOR_GM6020` / `MOTOR_NONE`，
  默认 `MOTOR_M3508`。
- `reverse`：是否反向，默认 `false`。
- `feedback_id`：电机反馈帧 ID（见上表），默认 `0x201`。

## 使用

```sh
xrobot module add QDU-Robomaster/RMMotor
xrobot setup
xrobot instance add QDU-Robomaster/RMMotor
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `can_bus` 填为 BSP 中用 `XR_REGISTER` 注册的 CAN 对象名：

```yaml
modules:
  - module: QDU-Robomaster/RMMotor
    id: rmmotor_0
    args:
      - can_bus: can1
      - param:
          model: RMMotor::Model::MOTOR_M3508
          reverse: 'false'
          feedback_id: '0x201'
```

BSP 侧：

```cpp
XR_REGISTER(can1, LibXR::CAN);
```

其他模块的 `Motor&` / `RMMotor&` 参数（如 `Chassis` 的轮电机、发射机构的摩擦轮）直接填写本实例的
id（此处为 `rmmotor_0`），本实例须在它们之前列出。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/RMMotor`
（在 BSP 中）打印当前的构造函数。
