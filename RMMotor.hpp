#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: RoboMaster电机驱动模块
constructor_args:
  - param:
      model: RMMotor::Model::MOTOR_M3508
      reverse: false
      feedback_id: 0x201
      can_bus_name: can1
template_args: []
required_hardware:
  - can
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/* RMMotor id */
/* id     feedback id     control id */
/* 1-4    0x205 to 0x208  0x1fe */
/* 5-6    0x209 to 0x20B  0x2fe */
#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1fe)
#define GM6020_CTRL_ID_EXTAND (0x2fe)

/* id     feedback id		  control id */
/* 1-4		0x201 to 0x204  0x200 */
/* 5-6		0x205 to 0x208  0x1ff */
#define M3508_M2006_FB_ID_BASE (0x201)
#define M3508_M2006_FB_ID_EXTAND (0x205)
#define M3508_M2006_CTRL_ID_BASE (0x200)
#define M3508_M2006_CTRL_ID_EXTAND (0x1ff)
#define M3508_M2006_ID_SETTING_ID (0x700)

#define MOTOR_CTRL_ID_NUMBER (4)

#define GM6020_MAX_ABS_LSB (16384)
#define M3508_MAX_ABS_LSB (16384)
#define M2006_MAX_ABS_LSB (10000)

/* 电机最大电流绝对值 */
#define GM6020_MAX_ABS_CUR (3)
#define M3508_MAX_ABS_CUR (20)
#define M2006_MAX_ABS_CUR (10)

#define MOTOR_ENC_RES (8192)  /* 电机编码器分辨率 */
#define MOTOR_CUR_RES (16384) /* 电机转矩电流分辨率 */

class RMMotor : public LibXR::Application {
 public:
  /**
   * @brief 电机型号
   */
  enum class Model : uint8_t {
    MOTOR_NONE = 0,
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
  };

  /**
   * @brief 电机参数
   */
  struct Param {
    Model model;
    bool reverse;
    uint16_t feedback_id;
    const char* can_bus_name;
  };

  struct ConfigParam {
    uint32_t id_feedback;
    uint32_t id_control;
  };

  struct Feedback {
    float rotor_abs_angle = 0.0f;
    float rotor_rotation_speed = 0.0f;
    float torque_current = 0.0f;
    float temp = 0.0f;
  };

  static inline uint8_t motor_tx_buff_[MOTOR_CTRL_ID_NUMBER][8]{};
  static inline uint8_t motor_tx_flag_[MOTOR_CTRL_ID_NUMBER]{};
  static inline uint8_t motor_tx_map_[MOTOR_CTRL_ID_NUMBER]{};

  /**
   * @brief RMMotor 类的构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param param 电机参数
   */
  RMMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const Param& param)
      : param_(param),
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
    UNUSED(app);
    switch (param_.model) {
      case Model::MOTOR_M2006:
      case Model::MOTOR_M3508:
        if (param_.feedback_id >= 0x201 && param_.feedback_id <= 0x204) {
          config_param_.id_control = M3508_M2006_CTRL_ID_BASE;
          config_param_.id_feedback = param_.feedback_id;
        } else if (param_.feedback_id >= 0x205 && param_.feedback_id <= 0x208) {
          config_param_.id_control = M3508_M2006_CTRL_ID_EXTAND;
          config_param_.id_feedback = param_.feedback_id;
        }
        break;

      case Model::MOTOR_GM6020:
        if (param_.feedback_id >= 0x205 && param_.feedback_id <= 0x208) {
          config_param_.id_control = GM6020_CTRL_ID_BASE;
          config_param_.id_feedback = param_.feedback_id;
        } else if (param_.feedback_id >= 0x209 && param_.feedback_id <= 0x20B) {
          config_param_.id_control = GM6020_CTRL_ID_EXTAND;
          config_param_.id_feedback = param_.feedback_id;
        }
        break;

      default:
        config_param_.id_control = 0;
        config_param_.id_feedback = 0;
        break;
    }

    uint8_t motor_num = 0;
    uint8_t motor_index = 0;

    switch (config_param_.id_control) {
      case M3508_M2006_CTRL_ID_BASE:
        motor_index = 0;
        motor_num = config_param_.id_feedback - M3508_M2006_FB_ID_BASE;
        break;
      case M3508_M2006_CTRL_ID_EXTAND:
        motor_index = 1;
        motor_num = config_param_.id_feedback - M3508_M2006_FB_ID_EXTAND;
        break;
      case GM6020_CTRL_ID_BASE:
        motor_index = 2;
        motor_num = config_param_.id_feedback - GM6020_FB_ID_BASE;
        break;
      case GM6020_CTRL_ID_EXTAND:
        motor_index = 3;
        motor_num = config_param_.id_feedback - GM6020_FB_ID_EXTAND;
        break;
      default:
        motor_index = 0;
        motor_num = 0;
        break;
    }

    index_ = motor_index;
    num_ = motor_num;

    motor_tx_map_[motor_index] |= (1 << motor_num);

    memset(motor_tx_buff_[index_], 0, sizeof(motor_tx_buff_[index_]));

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, RMMotor* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, config_param_.id_feedback,
                   config_param_.id_feedback);
  }

  bool Update() {
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      this->Decode(pack);
    }
    return true;
  }

  /**
   * @brief 解码来自CAN总线的电机反馈数据包
   * @param pack 包含电机反馈数据的CAN数据包
   */
  void Decode(LibXR::CAN::ClassicPack& pack) {
    uint16_t raw_angle =
        static_cast<uint16_t>((pack.data[0] << 8) | pack.data[1]);
    int16_t raw_current =
        static_cast<int16_t>((pack.data[4] << 8) | pack.data[5]);

    feedback_.rotor_abs_angle = static_cast<float>(raw_angle) / MOTOR_ENC_RES *
                                static_cast<float>(M_2PI);
    feedback_.rotor_rotation_speed =
        static_cast<int16_t>((pack.data[2] << 8) | pack.data[3]);
    feedback_.torque_current =
        static_cast<float>(raw_current) * M3508_MAX_ABS_CUR / MOTOR_CUR_RES;
    feedback_.temp = pack.data[6];
  }

  /**
   * @brief 获取电机扭矩常数
   * @return float 电机扭矩常数(N·m/A)
   */
  float KGetTorque() {
    switch (param_.model) {
      case Model::MOTOR_M2006:
        return 0.005f;
      case Model::MOTOR_M3508:
        return 0.0156224f;
      case Model::MOTOR_GM6020:
        return 0.741f;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 获取电机最大允许电流
   * @return float 最大电流值(A)
   */
  float GetCurrentMAX() {
    switch (param_.model) {
      case Model::MOTOR_M2006:
        return M2006_MAX_ABS_CUR;
      case Model::MOTOR_M3508:
        return M3508_MAX_ABS_CUR;
      case Model::MOTOR_GM6020:
        return GM6020_MAX_ABS_CUR;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 获取编码阈值
   * @return float 编码阈值
   */
  float GetLSB() {
    switch (param_.model) {
      case Model::MOTOR_M2006:
        return M2006_MAX_ABS_LSB;
      case Model::MOTOR_M3508:
      case Model::MOTOR_GM6020:
        return GM6020_MAX_ABS_LSB;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 获取电机转速(RPM)
   * @return float 电机转速，如果设置了反向则返回负值
   */
  float GetRPM() {
    if (param_.reverse) {
      return -feedback_.rotor_rotation_speed;
    } else {
      return feedback_.rotor_rotation_speed;
    }
  }

  /**
   * @brief 获取电机角速度(rad/s)
   * @return float 电机角速度，如果设置了反向则返回负值
   */
  float GetOmega() {
    if (param_.reverse) {
      return -feedback_.rotor_rotation_speed / 184.6153f;
    } else {
      return feedback_.rotor_rotation_speed / 184.6153f;
    }
  }

  /**      // 处理错误情况，尽管在此场景下几乎不可能发生
   * @brief 获取电机扭矩电流
   * @return float 扭矩电流(A)，如果设置了反向则返回负值
   */
  float GetCurrent() {
    if (param_.reverse) {
      return -feedback_.torque_current;
    } else {
      return feedback_.torque_current;
    }
  }

  /**
   * @brief 获取电机温度
   * @return float 电机温度(°C)
   */
  float GetTemp() { return feedback_.temp; }

  /**
   * @brief 获取电机转子绝对角度
   * @return float 转子绝对角度(rad)
   */
  float GetAngle() { return LibXR::CycleValue<float>(feedback_.rotor_abs_angle); }
  /**
   * @brief 设置电机的扭矩控制指令
   * @details 将归一化的输出值转换为16位整数指令，存入共享发送缓冲区。
   *          当同一控制ID下的所有电机都更新指令后，触发CAN报文发送。
   * @param torque 归一化的电机扭矩输出值，范围 [-6.0, 6.0](M3508)
   */
  void TorqueControl(float torque, float reduction_ratio) {
    if (feedback_.temp > 75.0f) {
      torque = 0.0f;
      XR_LOG_WARN("motor %d high temperature detected", param_.feedback_id);
    }

    float output =
        std::clamp(torque / reduction_ratio / KGetTorque() / GetCurrentMAX(),
                   -1.0f, 1.0f) *
        GetLSB();

    if (param_.reverse) {
      output = -output;
    }

    int16_t ctrl_cmd = static_cast<int16_t>(output);
    motor_tx_buff_[index_][2 * num_] =
        static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
    motor_tx_buff_[index_][2 * num_ + 1] =
        static_cast<uint8_t>(ctrl_cmd & 0xFF);
    motor_tx_flag_[index_] |= 1 << (num_);

    if (((~motor_tx_flag_[index_]) & (motor_tx_map_[index_])) == 0) {
      SendData();
    }
  }

  /**
   * @brief 设置电机的输出轴转速控制指令
   * @details 将归一化的输出值转换为16位整数指令，存入共享发送缓冲区。
   *          当同一控制ID下的所有电机都更新指令后，触发CAN报文发送。
   * @param out 归一化的电机转速输出值，范围 [-1.0, 1.0]
   */
  void CurrentControl(float out) {
    if (feedback_.temp > 75.0f) {
      out = 0.0f;
      XR_LOG_WARN("motor %d high temperature detected", param_.feedback_id);
    }

    out = std::clamp(out, -1.0f, 1.0f);
    float output = std::clamp(out * GetLSB(), -GetLSB(), GetLSB());

    if (param_.reverse) {
      output = -output;
    }

    int16_t ctrl_cmd = static_cast<int16_t>(output);
    motor_tx_buff_[index_][2 * num_] =
        static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
    motor_tx_buff_[index_][2 * num_ + 1] =
        static_cast<uint8_t>(ctrl_cmd & 0xFF);
    motor_tx_flag_[index_] |= 1 << (num_);

    if (((~motor_tx_flag_[index_]) & (motor_tx_map_[index_])) == 0) {
      SendData();
    }
  }


  /**
   * @brief 发送打包好的CAN控制报文
   * @details 从共享缓冲区拷贝数据到CAN包，通过CAN总线发送，并清零发送标志位。
   * @return bool 总是返回 true
   */
  bool SendData() {
    LibXR::CAN::ClassicPack tx_pack{};

    tx_pack.id = config_param_.id_control;
    tx_pack.type = LibXR::CAN::Type::STANDARD;

    LibXR::Memory::FastCopy(tx_pack.data, motor_tx_buff_[index_], sizeof(tx_pack.data));

    can_->AddMessage(tx_pack);

    motor_tx_flag_[index_] = 0;

    memset(motor_tx_buff_[index_], 0, sizeof(motor_tx_buff_[index_]));

    return true;
  }

  /**
   * @brief 将电机反馈数据清零，用于电机离线状态
   */
  void Offline() {
    feedback_.rotor_abs_angle = 0.0f;
    feedback_.rotor_rotation_speed = 0.0f;
    feedback_.torque_current = 0.0f;
    feedback_.temp = 0.0f;
  }

  void Relax() { this->CurrentControl(0.0f); }

  void OnMonitor() override {}

 private:
  /**
   * @brief CAN 接收回调的静态包装函数
   * @details
   * 将接收到的CAN数据包推入无锁队列中，供后续处理。如果队列已满，则丢弃最旧的数据包。
   * @param in_isr 指示是否在中断服务程序中调用
   * @param self 用户提供的参数，这里是 RMMotor 实例的指针
   * @param pack 接收到的 CAN 数据包
   */
  static void RxCallback(bool in_isr, RMMotor* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }

  uint8_t index_;
  uint8_t num_;

  Param param_;
  ConfigParam config_param_;
  Feedback feedback_;

  LibXR::CAN* can_;
  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};
};
