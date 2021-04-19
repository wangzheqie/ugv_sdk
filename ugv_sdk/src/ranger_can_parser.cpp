/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-19  17:1:21
 * @FileName  : ranger_can_parser.cpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#include "ugv_sdk/ranger/ranger_can_parser.hpp"
#include <string>
#include <cstring>

static void EncodeRangerMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame);
static void EncodeRangerLightControlMsgToCAN(const LightControlMessage *msg, struct can_frame *tx_frame);


bool DecodeRangerMsgFromCAN(const struct can_frame *rx_frame, RangerMessage *msg)
{
    msg->type = RangerMsgNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_STATUS_ID:
    {
        msg->type = RangerMotionStatusMsg;
        // msg->motion_status_msg.id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        memcpy(msg->body.motion_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_STATUS_ID:
    {
        msg->type = RangerLightStatusMsg;
        // msg->light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        memcpy(msg->body.light_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msg->type = RangerSystemStatusMsg;
        // msg->system_status_msg.id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        memcpy(msg->body.system_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_DRIVER_STATUS_ID:
    {
        msg->type = RangerMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR1_ID;
        memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_DRIVER_STATUS_ID:
    {
        msg->type = RangerMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR2_ID;
        memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR3_DRIVER_STATUS_ID:
    {
        msg->type = RangerMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR3_ID;
        memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR4_DRIVER_STATUS_ID:
    {
        msg->type = RangerMotorDriverStatusMsg;
        // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
        msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR4_ID;
        memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_CMD_ID:
    {
        msg->type = RangerMotionControlMsg;
        // msg->motion_control_msg.id = CAN_MSG_MOTION_CONTROL_CMD_ID;
        memcpy(msg->body.motion_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_CONTROL_CMD_ID:
    {
        msg->type = RangerLightControlMsg;
        // msg->light_control_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        memcpy(msg->body.light_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}

void EncodeRangerMsgToCAN(const RangerMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->type)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case RangerMotionStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case RangerLightStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.light_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case RangerSystemStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.system_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case RangerMotorDriverStatusMsg:
    {
        if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR1_ID)
            tx_frame->can_id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        else if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR2_ID)
            tx_frame->can_id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        else if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR3_ID)
            tx_frame->can_id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        else if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR4_ID)
            tx_frame->can_id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motor_driver_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case RangerMotionControlMsg:
    {
        EncodeRangerMotionControlMsgToCAN(&(msg->body.motion_control_msg), tx_frame);
        break;
    }
    case RangerLightControlMsg:
    {
        EncodeRangerLightControlMsgToCAN(&(msg->body.light_control_msg), tx_frame);
        break;
    }
    default:
        break;
    }
    tx_frame->data[7] = CalcRangerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeRangerMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_MOTION_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcRangerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeRangerLightControlMsgToCAN(const LightControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcRangerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

uint8_t CalcRangerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
}
