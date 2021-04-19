/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-19  16:51:51
 * @FileName  : ranger_can_parser.hpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#ifndef RANGER_CAN_PARSER_HPP
#define RANGER_CAN_PARSER_HPP
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ugv_sdk/ranger/ranger_protocol.hpp"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

bool DecodeRangerMsgFromCAN(const struct can_frame *rx_frame,
                            RangerMessage *msg);
void EncodeRangerMsgToCAN(const RangerMessage *msg, struct can_frame *tx_frame);

uint8_t CalcRangerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif  // RANGER_CAN_PARSER_HPP
