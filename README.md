# RMMotor

## 1. 模块作用
RoboMaster 电机驱动模块。实现 CAN 收发、反馈解析和 Motor 接口。
Manifest 描述：RoboMaster电机驱动模块

## 2. 主要函数说明
1. Decode: 解析电机反馈数据帧。
2. SendData: 打包并发送控制数据。
3. TorqueControl / CurrentControl: 力矩和电流控制路径。
4. Update / GetFeedback / Control: 标准驱动接口。
5. GetCurrentMAX / GetLSB: 模型相关换算参数。

## 3. 接入步骤
1. 添加模块并配置 model、feedback_id、can_bus_name、reverse。
2. 上电先确认反馈在线，再开放控制输出。
3. 在上层通过 Motor* 调用该驱动。

标准命令流程：
    xrobot_add_mod RMMotor --instance-id rmmotor
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: RMMotor
entry_header: Modules/RMMotor/RMMotor.hpp
constructor_args:
  - param:
      model: RMMotor::Model::MOTOR_M3508
      reverse: false
      feedback_id: 0x201
      can_bus_name: can1
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
  - can

Depends:
[]

## 6. 代码入口
Modules/RMMotor/RMMotor.hpp
