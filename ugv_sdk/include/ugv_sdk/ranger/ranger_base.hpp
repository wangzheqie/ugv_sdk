/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-19  16:51:45
 * @FileName  : ranger_base.hpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#ifndef RANGER_BASE_HPP
#define RANGER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "ugv_sdk/mobile_base.hpp"

#include "ugv_sdk/ranger/ranger_protocol.hpp"
#include "ugv_sdk/ranger/ranger_can_parser.hpp"
//#include "ugv_sdk/ranger/ranger_uart_parser.h"
#include "ugv_sdk/ranger/ranger_types.hpp"



namespace westonrobot {
class RangerBase : public MobileBase {
 public:
  RangerBase(bool is_ranger_mini = false)
      : MobileBase(), is_ranger_mini_(is_ranger_mini){};
  ~RangerBase() = default;

 public:
  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel,
                        RangerMotionCmd::FaultClearFlag fault_clr_flag =
                            RangerMotionCmd::FaultClearFlag::NO_FAULT);

  // light control
  void SetLightCommand(RangerLightCmd cmd);
  void DisableLightCmdControl();

  // get robot state
  RangerState GetRangerState();

 private:
  bool is_ranger_mini_ = false;

  // serial port buffer
  uint8_t tx_cmd_len_;
  uint8_t tx_buffer_[RANGER_CMD_BUF_LEN];

  // cmd/status update related variables
  std::mutex ranger_state_mutex_;
  std::mutex motion_cmd_mutex_;
  std::mutex light_cmd_mutex_;

  RangerState ranger_state_;
  RangerMotionCmd current_motion_cmd_;
  RangerLightCmd current_light_cmd_;

  bool light_ctrl_enabled_ = false;
  bool light_ctrl_requested_ = false;

  // internal functions
  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame);
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received);

  void SendMotionCmd(uint8_t count);
  void SendLightCmd(uint8_t count);
  void NewStatusMsgReceivedCallback(const RangerMessage &msg);

 public:
  static void UpdateRangerState(const RangerMessage &status_msg,
                               RangerState &state);
};
}  // namespace westonrobot

#endif // RANGER_BASE_HPP
