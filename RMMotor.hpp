#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: RoboMaster motor
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

#include "Motor.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "mutex.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"

/* RMMotor id */
/* id     feedback id     control id */
/* 1-4    0x205 to 0x208  0x1fe */
/* 5-6    0x209 to 0x20B  0x2fe */
#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1fe)
#define GM6020_CTRL_ID_EXTAND (0x2fe)

/* id     feedback id     control id */
/* 1-4    0x201 to 0x204  0x200 */
/* 5-6    0x205 to 0x208  0x1ff */
#define M3508_M2006_FB_ID_BASE (0x201)
#define M3508_M2006_FB_ID_EXTAND (0x205)
#define M3508_M2006_CTRL_ID_BASE (0x200)
#define M3508_M2006_CTRL_ID_EXTAND (0x1ff)
#define M3508_M2006_ID_SETTING_ID (0x700)

#define MOTOR_CTRL_ID_NUMBER (4)

#define GM6020_MAX_ABS_LSB (16384)
#define M3508_MAX_ABS_LSB (16384)
#define M2006_MAX_ABS_LSB (10000)

#define GM6020_MAX_ABS_CUR (3)
#define M3508_MAX_ABS_CUR (20)
#define M2006_MAX_ABS_CUR (10)

#define MOTOR_ENC_RES (8192)
#define MOTOR_CUR_RES (16384)

/**
 * @brief RoboMaster 电机驱动模块
 *
 * @details
 * 该模块负责：
 * - 解析不同型号电机的反馈 ID 与控制 ID 映射关系
 * - 订阅对应反馈 CAN 帧并解码为统一的电机反馈格式
 * - 将同一 CAN 总线、同一控制 ID 组内的多个电机命令拼成一帧再发送
 *
 * 当前拼包语义为：组内所有已注册成员都在本轮更新过一次后，才触发一次发送。
 */
class RMMotor : public LibXR::Application, public Motor {
 public:
  /**
   * @brief RoboMaster 电机型号
   */
  enum class Model : uint8_t {
    MOTOR_NONE = 0,
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
  };

  /**
   * @brief 模块构造参数
   */
  struct Param {
    Model model;           ///< 电机型号
    bool reverse;          ///< 是否反向解释反馈并反向输出控制
    uint16_t feedback_id;  ///< 电机反馈 CAN ID
    const char*
        can_bus_name;  ///< CAN 硬件别名，用于从 HardwareContainer 查找总线
  };

  /**
   * @brief 由反馈 ID 推导出的控制配置
   */
  struct ConfigParam {
    uint32_t id_feedback;  ///< 反馈帧 ID
    uint32_t id_control;   ///< 控制帧 ID
  };

  /**
   * @brief 单个控制组的拼包状态
   *
   * @details
   * 一个控制组对应一个 8 字节发送帧，最多容纳 4 个电机，每个电机占 2 字节槽位。
   */
  struct MotorGroupState {
    uint8_t tx_buff[8]{};    ///< 当前拼包缓存
    uint8_t pending_mask{};  ///< 本轮已写入命令的成员位图
    uint8_t group_mask{};    ///< 当前组内已注册成员位图
    LibXR::Mutex mutex;      ///< 保护本组拼包状态
  };

  /**
   * @brief 单条 CAN 总线的共享状态
   *
   * @details
   * 以 `LibXR::CAN*` 作为总线身份，而不是以字符串别名作为身份。
   * 同一个 CAN 对象的多个 alias 会落到同一个 BusState。
   */
  struct BusState {
    LibXR::CAN* can{};                               ///< 对应的 CAN 对象
    MotorGroupState groups[MOTOR_CTRL_ID_NUMBER]{};  ///< 4 个控制 ID 组的状态
    BusState* next{};                                ///< 注册表单链表下一项
  };

  /**
   * @brief 构造 RoboMaster 电机实例
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param param 模块构造参数
   *
   * @details
   * 构造时会完成：
   * - 通过 `can_bus_name` 查找对应的 `LibXR::CAN`
   * - 根据电机型号与反馈 ID 推导控制组和槽位编号
   * - 为所在 `(can, control_id)` 组注册成员位
   * - 注册反馈帧接收回调
   */
  RMMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const Param& param)
      : param_(param),
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
    UNUSED(app);
    reverse_flag_ = param_.reverse ? -1.0f : 1.0f;

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
    bus_state_ = &GetOrCreateBusState(can_);

    {
      auto& group_state = GetMotorGroupState();
      LibXR::Mutex::LockGuard guard(group_state.mutex);
      if (group_state.group_mask == 0U) {
        memset(group_state.tx_buff, 0, sizeof(group_state.tx_buff));
      }
      group_state.pending_mask = 0U;
      group_state.group_mask |= static_cast<uint8_t>(1U << num_);
    }

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, RMMotor* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, config_param_.id_feedback,
                   config_param_.id_feedback);
  }

  /**
   * @brief 使能电机输出
   *
   * @note RoboMaster 电机常规控制不需要独立使能命令，此处为空实现。
   */
  void Enable() override { return; }

  /**
   * @brief 失能电机输出
   *
   * @details 通过下发 0 电流实现。
   */
  void Disable() override { CurrentControl(0.0f); }

  /**
   * @brief 松开电机
   *
   * @details 通过下发 0 电流实现。
   */
  void Relax() override { CurrentControl(0.0f); }

  /**
   * @brief 更新电机反馈
   *
   * @return
   * - `ErrorCode::OK`：本次至少收到并解码了一帧反馈
   * - `ErrorCode::NO_RESPONSE`：连续无反馈次数超过阈值
   */
  ErrorCode Update() override {
    LibXR::CAN::ClassicPack pack;
    bool get_feedback = false;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      Decode(pack);
      get_feedback = true;
    }

    if (get_feedback) {
      no_response_count_ = 0U;
      return ErrorCode::OK;
    }

    if (no_response_count_ <= NO_RESPONSE_THRESHOLD) {
      ++no_response_count_;
    }

    return no_response_count_ > NO_RESPONSE_THRESHOLD ? ErrorCode::NO_RESPONSE
                                                      : ErrorCode::OK;
  }

  /**
   * @brief 获取当前反馈
   * @return 当前反馈结构体引用
   */
  const Feedback& GetFeedback() override { return feedback_; }

  /**
   * @brief 下发控制命令
   * @param cmd 电机控制命令
   */
  void Control(const MotorCmd& cmd) override {
    switch (cmd.mode) {
      case ControlMode::MODE_TORQUE:
        TorqueControl(cmd.torque, cmd.reduction_ratio);
        break;
      case ControlMode::MODE_CURRENT:
        CurrentControl(cmd.velocity);
        break;
      default:
        break;
    }
  }

  /**
   * @brief 清除错误
   *
   * @note RoboMaster 电机协议未在此模块中实现独立清错命令。
   */
  void ClearError() override { return; }

  /**
   * @brief 保存零点
   *
   * @note RoboMaster 电机协议未在此模块中实现零点保存命令。
   */
  void SaveZeroPoint() override { return; }

  /**
   * @brief 周期监控回调
   *
   * @note 当前实现未使用该钩子。
   */
  void OnMonitor() override {}

 private:
  static constexpr uint16_t NO_RESPONSE_THRESHOLD = 255U;

  uint8_t index_{};  ///< 控制组索引，对应不同 control ID
  uint8_t num_{};    ///< 当前电机在 8 字节控制帧中的槽位编号

  float reverse_flag_ = 1.0f;  ///< 方向系数，正向为 1，反向为 -1

  Param param_;                   ///< 构造参数副本
  ConfigParam config_param_{};    ///< 反馈/控制 ID 配置
  Motor::Feedback feedback_{};    ///< 最近一次解码得到的反馈
  uint16_t no_response_count_{};  ///< 连续无反馈计数

  LibXR::CAN* can_;        ///< 当前实例所属 CAN 总线
  BusState* bus_state_{};  ///< 当前实例所属总线共享状态
  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};  ///< 接收队列

  static inline LibXR::Mutex
      bus_state_registry_mutex_{};                     ///< 总线状态注册表互斥锁
  static inline BusState* bus_state_registry_head_{};  ///< 总线状态注册表头指针

  /**
   * @brief 发送已经打包完成的 CAN 帧
   * @param tx_pack 待发送的控制帧
   * @return `true` 表示成功加入底层 CAN 发送队列
   */
  bool SendData(const LibXR::CAN::ClassicPack& tx_pack) {
    return can_->AddMessage(tx_pack) == ErrorCode::OK;
  }

  /**
   * @brief 获取当前实例所在控制组的共享状态
   * @return 控制组状态引用
   */
  MotorGroupState& GetMotorGroupState() {
    ASSERT(bus_state_ != nullptr);
    return bus_state_->groups[index_];
  }

  /**
   * @brief CAN 接收回调的静态包装函数
   * @param in_isr 是否在中断上下文中调用
   * @param self 当前电机实例
   * @param pack 接收到的 CAN 帧
   *
   * @details
   * 为了始终保留最新反馈，若队列已满会先弹出最旧数据，再压入最新数据。
   */
  static void RxCallback(bool in_isr, RMMotor* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }

  /**
   * @brief 解码 RoboMaster 电机反馈帧
   * @param pack 反馈 CAN 帧
   */
  void Decode(LibXR::CAN::ClassicPack& pack) {
    uint16_t raw_angle =
        static_cast<uint16_t>((pack.data[0] << 8) | pack.data[1]);
    int16_t raw_velocity =
        static_cast<int16_t>((pack.data[2] << 8) | pack.data[3]);
    int16_t raw_current =
        static_cast<int16_t>((pack.data[4] << 8) | pack.data[5]);
    uint8_t raw_temp = pack.data[6];

    if (param_.reverse) {
      feedback_.position = -static_cast<float>(raw_angle) / MOTOR_ENC_RES *
                           static_cast<float>(M_2PI);
      feedback_.velocity = static_cast<float>(-raw_velocity);
    } else {
      feedback_.position = static_cast<float>(raw_angle) / MOTOR_ENC_RES *
                           static_cast<float>(M_2PI);
      feedback_.velocity = static_cast<float>(raw_velocity);
    }

    feedback_.abs_angle = LibXR::CycleValue<float>(feedback_.position);
    feedback_.omega = feedback_.velocity * (static_cast<float>(M_2PI) / 60.0f);
    feedback_.torque = static_cast<float>(raw_current) * KGetTorque() *
                       GetCurrentMAX() / MOTOR_CUR_RES;
    feedback_.temp = static_cast<float>(raw_temp);
    feedback_.state = 1;
  }

  /**
   * @brief 将当前电机命令写入组帧缓存，并在满足条件时发送整帧
   * @param ctrl_cmd 当前电机对应的 16 位控制量
   *
   * @details
   * 槽位映射规则为 `2 * num_` 和 `2 * num_ + 1`。
   * 仅当 `pending_mask == group_mask` 时，才会发送当前组的 8 字节控制帧。
   */
  void PackAndSend(int16_t ctrl_cmd) {
    const uint8_t motor_bit = static_cast<uint8_t>(1U << num_);
    bool should_send = false;
    LibXR::CAN::ClassicPack tx_pack{};

    {
      auto& group_state = GetMotorGroupState();
      LibXR::Mutex::LockGuard guard(group_state.mutex);

      group_state.tx_buff[2 * num_] =
          static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
      group_state.tx_buff[2 * num_ + 1] = static_cast<uint8_t>(ctrl_cmd & 0xFF);
      group_state.pending_mask |= motor_bit;

      if (group_state.group_mask != 0U &&
          group_state.pending_mask == group_state.group_mask) {
        tx_pack.id = config_param_.id_control;
        tx_pack.type = LibXR::CAN::Type::STANDARD;
        tx_pack.dlc = 8;
        LibXR::Memory::FastCopy(tx_pack.data, group_state.tx_buff,
                                sizeof(tx_pack.data));

        group_state.pending_mask = 0U;
        should_send = true;
      }
    }

    if (should_send) {
      SendData(tx_pack);
    }
  }

  /**
   * @brief 力矩控制
   * @param torque 目标输出力矩
   * @param reduction_ratio 减速比
   */
  void TorqueControl(float torque, float reduction_ratio) {
    if (feedback_.temp > 75.0f) {
      torque = 0.0f;
      XR_LOG_WARN("motor %d high temperature detected", param_.feedback_id);
    }

    float output =
        std::clamp(torque / reduction_ratio / KGetTorque() / GetCurrentMAX(),
                   -1.0f, 1.0f) *
        GetLSB() * reverse_flag_;

    int16_t ctrl_cmd = static_cast<int16_t>(output);
    PackAndSend(ctrl_cmd);
  }

  /**
   * @brief 电流控制
   * @param out 归一化输出，范围通常为 [-1.0, 1.0]
   */
  void CurrentControl(float out) {
    if (feedback_.temp > 75.0f) {
      out = 0.0f;
      XR_LOG_WARN("motor %d high temperature detected", param_.feedback_id);
    }

    out = std::clamp(out, -1.0f, 1.0f);
    float output =
        std::clamp(out * GetLSB(), -GetLSB(), GetLSB()) * reverse_flag_;

    int16_t ctrl_cmd = static_cast<int16_t>(output);
    PackAndSend(ctrl_cmd);
  }

  /**
   * @brief 获取电机力矩常数
   * @return 对应型号的力矩常数
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
   * @brief 获取电机最大电流
   * @return 对应型号的最大绝对电流
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
   * @brief 获取控制量 LSB 上限
   * @return 对应型号的控制量满量程
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
   * @brief 在总线状态注册表中查找指定 CAN 的共享状态
   * @param can CAN 对象指针
   * @return 找到则返回对应状态指针，否则返回 `nullptr`
   */
  static BusState* FindBusState(LibXR::CAN* can) {
    BusState* state = bus_state_registry_head_;
    while (state != nullptr) {
      if (state->can == can) {
        return state;
      }
      state = state->next;
    }
    return nullptr;
  }

  /**
   * @brief 获取或创建指定 CAN 对应的共享总线状态
   * @param can CAN 对象指针
   * @return 总线状态引用
   *
   * @details
   * 该函数以 `LibXR::CAN*` 作为总线身份标识，而不是以 `can_bus_name` 字符串
   * 作为标识。这样同一 CAN 对象的多个 alias 会共享同一个拼包状态。
   *
   * 注册表采用单链表保存，创建策略为只增不删，符合本项目初始化阶段分配、
   * 运行期不释放的使用方式。
   */
  static BusState& GetOrCreateBusState(LibXR::CAN* can) {
    LibXR::Mutex::LockGuard guard(bus_state_registry_mutex_);
    if (BusState* state = FindBusState(can); state != nullptr) {
      return *state;
    }
    auto* state = new BusState{};
    state->can = can;
    state->next = bus_state_registry_head_;
    bus_state_registry_head_ = state;
    return *state;
  }
};
