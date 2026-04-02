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

class RMMotor : public LibXR::Application, public Motor {
 public:
  enum class Model : uint8_t {
    MOTOR_NONE = 0,
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
  };

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

  struct MotorGroupState {
    uint8_t tx_buff[8]{};
    uint8_t pending_mask{};
    uint8_t group_mask{};
    LibXR::Mutex mutex;
  };

  struct BusState {
    LibXR::CAN* can{};
    MotorGroupState groups[MOTOR_CTRL_ID_NUMBER]{};
    BusState* next{};
  };

  static inline LibXR::Mutex bus_state_registry_mutex_{};
  static inline BusState* bus_state_registry_head_{};

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

  void Enable() override { return; }

  void Disable() override { CurrentControl(0.0f); }

  void Relax() override { CurrentControl(0.0f); }

  ErrorCode Update() override {
    LibXR::CAN::ClassicPack pack;
    bool get_feedback = false;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      Decode(pack);
      get_feedback = true;
    }
    return get_feedback ? ErrorCode::OK : ErrorCode::NO_RESPONSE;
  }

  const Feedback& GetFeedback() override { return feedback_; }

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

  void ClearError() override { return; }
  void SaveZeroPoint() override { return; }
  void OnMonitor() override {}

 private:
  uint8_t index_{};
  uint8_t num_{};

  float reverse_flag_ = 1.0f;

  Param param_;
  ConfigParam config_param_{};
  Motor::Feedback feedback_{};

  LibXR::CAN* can_;
  BusState* bus_state_{};
  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};

  bool SendData(const LibXR::CAN::ClassicPack& tx_pack) {
    return can_->AddMessage(tx_pack) == ErrorCode::OK;
  }

  MotorGroupState& GetMotorGroupState() {
    ASSERT(bus_state_ != nullptr);
    return bus_state_->groups[index_];
  }

  static void RxCallback(bool in_isr, RMMotor* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }

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

      // Send after every active motor in the control group has updated.
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
};
