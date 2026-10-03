#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: RoboMaster 电机驱动模块（M2006 / M3508 / GM6020）：CAN 收发、反馈解码与 Motor 接口 / RoboMaster motor driver Module (M2006 / M3508 / GM6020) with CAN transfer, feedback decoding and the Motor interface
depends:
- id: QDU-Robomaster/Motor
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include "Motor.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "mutex.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"

// GM6020: feedback ID 0x205-0x208 -> control ID 0x1FE, 0x209-0x20B -> control ID 0x2FE.
// GM6020：反馈 ID 0x205-0x208 对应控制 ID 0x1FE，0x209-0x20B 对应控制 ID 0x2FE。
#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1fe)
#define GM6020_CTRL_ID_EXTAND (0x2fe)

// M3508 / M2006: feedback ID 0x201-0x204 -> control ID 0x200,
//                feedback ID 0x205-0x208 -> control ID 0x1FF.
// M3508 / M2006：反馈 ID 0x201-0x204 对应控制 ID 0x200，
//                反馈 ID 0x205-0x208 对应控制 ID 0x1FF。
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
 * @brief RoboMaster 电机驱动模块（M2006 / M3508 / GM6020）。
 *        RoboMaster motor driver Module (M2006 / M3508 / GM6020).
 *
 * @details 根据型号与反馈 ID 确定控制 ID 与槽位，接收反馈 CAN 帧并解码为
 *          Motor::Feedback；同一 CAN 总线、同一控制 ID 的电机命令拼成一个 8 字节帧，
 *          组内所有已构造的电机在本轮都写入命令后发送。
 *          Determines the control ID and slot from the model and the feedback ID,
 *          receives the feedback CAN frames and decodes them into Motor::Feedback.
 *          Commands of motors on the same CAN bus with the same control ID are packed
 *          into one 8-byte frame, which is sent once every constructed motor of the
 *          group has written its command in the current round.
 */
class RMMotor : public Motor
{
 public:
  /**
   * @brief RoboMaster 电机型号。
   *        RoboMaster motor model.
   */
  enum class Model : uint8_t
  {
    MOTOR_NONE = 0,  ///< 未指定型号，控制帧 ID 为 0 Unspecified, control frame ID is 0
    MOTOR_M2006,     ///< M2006
    MOTOR_M3508,     ///< M3508
    MOTOR_GM6020,    ///< GM6020
  };

  /**
   * @brief 构造参数。
   *        Construction parameters.
   */
  struct Param
  {
    Model model;           ///< 电机型号 Motor model
    bool reverse;          ///< 为 true 时反馈与输出取反 True negates feedback and output
    uint16_t feedback_id;  ///< 电机反馈 CAN ID Motor feedback CAN ID
  };

  /**
   * @brief 由反馈 ID 推导出的控制配置。
   *        Control configuration derived from the feedback ID.
   */
  struct ConfigParam
  {
    uint32_t id_feedback;  ///< 反馈帧 ID Feedback frame ID
    uint32_t id_control;   ///< 控制帧 ID Control frame ID
  };

  /**
   * @brief 单个控制组的拼包状态。
   *        Packing state of one control group.
   *
   * @details 一个控制组对应一个 8 字节发送帧，最多容纳 4 个电机，每个电机占 2 字节槽位。
   *          One control group corresponds to one 8-byte transmit frame holding up to 4
   *          motors with a 2-byte slot each.
   */
  struct MotorGroupState
  {
    uint8_t tx_buff[8]{};    ///< 拼包缓存 Packing buffer
    uint8_t pending_mask{};  ///< 本轮已写入命令的成员位图 Members that wrote this round
    uint8_t group_mask{};    ///< 组内已构造的成员位图 Members constructed in the group
    LibXR::Mutex mutex;      ///< 保护本组拼包状态 Protects the packing state
  };

  /**
   * @brief 单条 CAN 总线的共享状态。
   *        Shared state of one CAN bus.
   *
   * @details 以 `LibXR::CAN*` 标识总线，指向同一 CAN 对象的电机共享同一个 BusState。
   *          The bus is identified by `LibXR::CAN*`; motors on the same CAN object
   *          share one BusState.
   */
  struct BusState
  {
    LibXR::CAN* can{};  ///< CAN 对象 CAN object
    MotorGroupState groups[MOTOR_CTRL_ID_NUMBER]{};  ///< 4 个控制组 4 control groups
    BusState* next{};  ///< 注册表链表的下一项 Next entry of the registry list
  };

  /**
   * @brief 构造 RMMotor，由型号与反馈 ID 确定控制组和槽位，并注册反馈帧接收回调。
   *        Construct RMMotor, determine the control group and slot from the model and the
   *        feedback ID, and register the feedback frame receive callback.
   *
   * @param can_bus 电机所在的 CAN 总线。
   *                CAN bus the motor is attached to.
   * @param param 构造参数。
   *              Construction parameters.
   */
  RMMotor(
      LibXR::CAN& can_bus,
      const Param& param = {
      .model = RMMotor::Model::MOTOR_M3508, .reverse = false, .feedback_id = 0x201}) : param_(param), can_(std::addressof(can_bus))
  {
    reverse_flag_ = param_.reverse ? -1.0f : 1.0f;

    switch (param_.model)
    {
      case Model::MOTOR_M2006:
      case Model::MOTOR_M3508:
        if (param_.feedback_id >= 0x201 && param_.feedback_id <= 0x204)
        {
          config_param_.id_control = M3508_M2006_CTRL_ID_BASE;
          config_param_.id_feedback = param_.feedback_id;
        }
        else if (param_.feedback_id >= 0x205 && param_.feedback_id <= 0x208)
        {
          config_param_.id_control = M3508_M2006_CTRL_ID_EXTAND;
          config_param_.id_feedback = param_.feedback_id;
        }
        break;

      case Model::MOTOR_GM6020:
        if (param_.feedback_id >= 0x205 && param_.feedback_id <= 0x208)
        {
          config_param_.id_control = GM6020_CTRL_ID_BASE;
          config_param_.id_feedback = param_.feedback_id;
        }
        else if (param_.feedback_id >= 0x209 && param_.feedback_id <= 0x20B)
        {
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

    switch (config_param_.id_control)
    {
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
      if (group_state.group_mask == 0U)
      {
        memset(group_state.tx_buff, 0, sizeof(group_state.tx_buff));
      }
      group_state.pending_mask = 0U;
      group_state.group_mask |= static_cast<uint8_t>(1U << num_);
    }

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, RMMotor* self, const LibXR::CAN::ClassicPack& pack)
        { RxCallback(in_isr, self, pack); }, this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, config_param_.id_feedback,
                   config_param_.id_feedback);
  }

  /**
   * @brief 使能电机，空实现。
   *        Enable the motor; empty implementation.
   */
  void Enable() override { return; }

  /**
   * @brief 失能电机，下发 0 电流。
   *        Disable the motor by sending 0 current.
   */
  void Disable() override { CurrentControl(0.0f); }

  /**
   * @brief 松开电机，下发 0 电流。
   *        Relax the motor by sending 0 current.
   */
  void Relax() override { CurrentControl(0.0f); }

  /**
   * @brief 解码队列中的反馈帧并更新反馈。
   *        Decode the queued feedback frames and update the feedback.
   *
   * @return 本次收到反馈或连续无反馈次数未超过 255 时为 `ErrorCode::OK`，否则为
   *         `ErrorCode::NO_RESPONSE`。
   *         `ErrorCode::OK` when feedback was received or the count of consecutive calls
   *         without feedback has not exceeded 255, otherwise `ErrorCode::NO_RESPONSE`.
   */
  LibXR::ErrorCode Update() override
  {
    LibXR::CAN::ClassicPack pack;
    bool get_feedback = false;
    while (recv_queue_.Pop(pack) == LibXR::ErrorCode::OK)
    {
      Decode(pack);
      get_feedback = true;
    }

    if (get_feedback)
    {
      no_response_count_ = 0U;
      return LibXR::ErrorCode::OK;
    }

    if (no_response_count_ <= NO_RESPONSE_THRESHOLD)
    {
      ++no_response_count_;
    }

    return no_response_count_ > NO_RESPONSE_THRESHOLD ? LibXR::ErrorCode::NO_RESPONSE
                                                      : LibXR::ErrorCode::OK;
  }

  /**
   * @brief 获取最近一次解码得到的反馈。
   *        Get the most recently decoded feedback.
   *
   * @return 反馈结构体引用。
   *         Reference to the feedback structure.
   */
  const Feedback& GetFeedback() override { return feedback_; }

  /**
   * @brief 下发控制命令，仅处理 `MODE_TORQUE` 与 `MODE_CURRENT`，其余模式被忽略。
   *        Send a control command; only `MODE_TORQUE` and `MODE_CURRENT` are handled,
   *        other modes are ignored.
   *
   * @param cmd 电机控制命令；`MODE_CURRENT` 的归一化电流取自 `cmd.velocity`。
   *            Motor control command; the normalized current of `MODE_CURRENT` is
   *            taken from `cmd.velocity`.
   */
  void Control(const MotorCmd& cmd) override
  {
    switch (cmd.mode)
    {
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
   * @brief 清除错误，空实现。
   *        Clear errors; empty implementation.
   */
  void ClearError() override { return; }

  /**
   * @brief 保存零点，空实现。
   *        Save the zero point; empty implementation.
   */
  void SaveZeroPoint() override { return; }

 private:
  static constexpr uint16_t NO_RESPONSE_THRESHOLD = 255U;

  uint8_t index_{};  ///< 控制组索引，对应不同控制 ID Control group index
  uint8_t num_{};    ///< 在 8 字节控制帧中的槽位编号 Slot number in the control frame

  float reverse_flag_ = 1.0f;  ///< 方向系数，正向 1，反向 -1 Direction factor, 1 or -1

  Param param_;                   ///< 构造参数副本 Copy of the construction parameters
  ConfigParam config_param_{};    ///< 反馈与控制 ID Feedback and control IDs
  Motor::Feedback feedback_{};    ///< 最近一次解码的反馈 Latest decoded feedback
  uint16_t no_response_count_{};  ///< 连续无反馈计数 Consecutive calls without feedback

  LibXR::CAN* can_;        ///< 所在 CAN 总线 CAN bus of the motor
  BusState* bus_state_{};  ///< 所在总线的共享状态 Shared state of the bus
  LibXR::MPMCQueue<LibXR::CAN::ClassicPack> recv_queue_{1};  ///< 接收队列 Receive queue

  static inline LibXR::Mutex bus_state_registry_mutex_{};  ///< 注册表互斥锁 Registry lock
  static inline BusState* bus_state_registry_head_{};      ///< 注册表头 Registry head

  /**
   * @brief 发送已打包的 CAN 帧。
   *        Send a packed CAN frame.
   *
   * @param tx_pack 待发送的控制帧。
   *                Control frame to send.
   * @return 成功加入 CAN 发送队列时为 true。
   *         True when the frame was queued for transmission.
   */
  bool SendData(const LibXR::CAN::ClassicPack& tx_pack)
  {
    return can_->AddMessage(tx_pack) == LibXR::ErrorCode::OK;
  }

  /**
   * @brief 获取所在控制组的共享状态。
   *        Get the shared state of the control group of this motor.
   *
   * @return 控制组状态引用。
   *         Reference to the control group state.
   */
  MotorGroupState& GetMotorGroupState()
  {
    ASSERT(bus_state_ != nullptr);
    return bus_state_->groups[index_];
  }

  /**
   * @brief CAN 接收回调。
   *        CAN receive callback.
   *
   * @param in_isr 是否在中断上下文中调用。
   *               Whether called from interrupt context.
   * @param self 电机实例。
   *             Motor instance.
   * @param pack 接收到的 CAN 帧。
   *             Received CAN frame.
   *
   * @details 队列已满时先弹出最旧的一帧，再压入新帧。
   *          When the queue is full the oldest frame is popped before the new one is
   *          pushed.
   */
  static void RxCallback(bool in_isr, RMMotor* self, const LibXR::CAN::ClassicPack& pack)
  {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != LibXR::ErrorCode::OK)
    {
      self->recv_queue_.Pop();
    }
  }

  /**
   * @brief 解码反馈帧。
   *        Decode a feedback frame.
   *
   * @param pack 反馈 CAN 帧。
   *             Feedback CAN frame.
   */
  void Decode(LibXR::CAN::ClassicPack& pack)
  {
    uint16_t raw_angle = static_cast<uint16_t>((pack.data[0] << 8) | pack.data[1]);
    int16_t raw_velocity = static_cast<int16_t>((pack.data[2] << 8) | pack.data[3]);
    int16_t raw_current = static_cast<int16_t>((pack.data[4] << 8) | pack.data[5]);
    uint8_t raw_temp = pack.data[6];

    if (param_.reverse)
    {
      feedback_.position = -static_cast<float>(raw_angle) / MOTOR_ENC_RES *
                           static_cast<float>(LibXR::TWO_PI);
      feedback_.velocity = static_cast<float>(-raw_velocity);
    }
    else
    {
      feedback_.position = static_cast<float>(raw_angle) / MOTOR_ENC_RES *
                           static_cast<float>(LibXR::TWO_PI);
      feedback_.velocity = static_cast<float>(raw_velocity);
    }

    feedback_.abs_angle = LibXR::CycleValue<float>(feedback_.position);
    feedback_.omega = feedback_.velocity * (static_cast<float>(LibXR::TWO_PI) / 60.0f);
    feedback_.torque =
        static_cast<float>(raw_current) * KGetTorque() * GetCurrentMAX() / MOTOR_CUR_RES;
    feedback_.temp = static_cast<float>(raw_temp);
    feedback_.state = 1;
  }

  /**
   * @brief 将命令写入组帧缓存，组内所有成员都已写入时发送整帧。
   *        Write the command into the group frame buffer and send the frame once all
   *        members of the group have written.
   *
   * @param ctrl_cmd 本电机的 16 位控制量。
   *                 16-bit control value of this motor.
   *
   * @details 命令占用字节 `2 * num_` 与 `2 * num_ + 1`；
   *          `pending_mask == group_mask` 时发送。
   *          The command occupies bytes `2 * num_` and `2 * num_ + 1`; the frame is
   *          sent when `pending_mask == group_mask`.
   */
  void PackAndSend(int16_t ctrl_cmd)
  {
    const uint8_t motor_bit = static_cast<uint8_t>(1U << num_);
    bool should_send = false;
    LibXR::CAN::ClassicPack tx_pack{};

    {
      auto& group_state = GetMotorGroupState();
      LibXR::Mutex::LockGuard guard(group_state.mutex);

      group_state.tx_buff[2 * num_] = static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
      group_state.tx_buff[2 * num_ + 1] = static_cast<uint8_t>(ctrl_cmd & 0xFF);
      group_state.pending_mask |= motor_bit;

      if (group_state.group_mask != 0U &&
          group_state.pending_mask == group_state.group_mask)
      {
        tx_pack.id = config_param_.id_control;
        tx_pack.type = LibXR::CAN::Type::STANDARD;
        tx_pack.dlc = 8;
        LibXR::Memory::FastCopy(tx_pack.data, group_state.tx_buff, sizeof(tx_pack.data));

        group_state.pending_mask = 0U;
        should_send = true;
      }
    }

    if (should_send)
    {
      SendData(tx_pack);
    }
  }

 public:
  /**
   * @brief 获取反馈角速度。
   *        Get the feedback angular velocity.
   *
   * @return 角速度，单位 rad/s。
   *         Angular velocity in rad/s.
   */
  float GetOmega() const { return feedback_.omega; }

  /**
   * @brief 力矩控制。
   *        Torque control.
   *
   * @param torque 输出轴目标力矩，单位 N·m。
   *               Target torque at the output shaft in N·m.
   * @param reduction_ratio 减速比。
   *                        Reduction ratio.
   */
  void TorqueControl(float torque, float reduction_ratio)
  {
    if (feedback_.temp > 75.0f)
    {
      torque = 0.0f;
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.feedback_id));
    }

    float output = std::clamp(torque / reduction_ratio / KGetTorque() / GetCurrentMAX(),
                              -1.0f, 1.0f) *
                   GetLSB() * reverse_flag_;

    int16_t ctrl_cmd = static_cast<int16_t>(output);
    PackAndSend(ctrl_cmd);
  }

 private:
  /**
   * @brief 电流控制。
   *        Current control.
   *
   * @param out 归一化电流，限幅到 [-1.0, 1.0]。
   *            Normalized current, clamped to [-1.0, 1.0].
   */
  void CurrentControl(float out)
  {
    if (feedback_.temp > 75.0f)
    {
      out = 0.0f;
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.feedback_id));
    }

    out = std::clamp(out, -1.0f, 1.0f);
    float output = std::clamp(out * GetLSB(), -GetLSB(), GetLSB()) * reverse_flag_;

    int16_t ctrl_cmd = static_cast<int16_t>(output);
    PackAndSend(ctrl_cmd);
  }

  /**
   * @brief 获取型号对应的力矩常数。
   *        Get the torque constant of the model.
   *
   * @return 力矩常数，单位 N·m/A。
   *         Torque constant in N·m/A.
   */
  float KGetTorque()
  {
    switch (param_.model)
    {
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
   * @brief 获取型号对应的最大电流。
   *        Get the maximum current of the model.
   *
   * @return 最大绝对电流，单位 A。
   *         Maximum absolute current in A.
   */
  float GetCurrentMAX()
  {
    switch (param_.model)
    {
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
   * @brief 获取型号对应的控制量满量程。
   *        Get the full-scale control value of the model.
   *
   * @return 控制量满量程。
   *         Full-scale control value.
   */
  float GetLSB()
  {
    switch (param_.model)
    {
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
   * @brief 在注册表中查找指定 CAN 的共享状态。
   *        Look up the shared state of a CAN object in the registry.
   *
   * @param can CAN 对象指针。
   *            CAN object pointer.
   * @return 对应状态指针，未找到时为 `nullptr`。
   *         Pointer to the state, `nullptr` when not found.
   */
  static BusState* FindBusState(LibXR::CAN* can)
  {
    BusState* state = bus_state_registry_head_;
    while (state != nullptr)
    {
      if (state->can == can)
      {
        return state;
      }
      state = state->next;
    }
    return nullptr;
  }

  /**
   * @brief 获取或创建指定 CAN 的共享总线状态。
   *        Get or create the shared bus state of a CAN object.
   *
   * @param can CAN 对象指针。
   *            CAN object pointer.
   * @return 总线状态引用。
   *         Reference to the bus state.
   *
   * @details 总线以 `LibXR::CAN*` 标识，指向同一 CAN 对象的电机共享同一个拼包状态。
   *          注册表为单链表，状态在初始化阶段分配，运行期不释放。
   *          The bus is identified by `LibXR::CAN*`, so motors on the same CAN object
   *          share one packing state. The registry is a singly linked list; states are
   *          allocated during initialization and are not released at run time.
   */
  static BusState& GetOrCreateBusState(LibXR::CAN* can)
  {
    LibXR::Mutex::LockGuard guard(bus_state_registry_mutex_);
    if (BusState* state = FindBusState(can); state != nullptr)
    {
      return *state;
    }
    auto* state = new BusState{};
    state->can = can;
    state->next = bus_state_registry_head_;
    bus_state_registry_head_ = state;
    return *state;
  }
};
