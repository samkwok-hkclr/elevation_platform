#ifndef COMMAND_GROUP_HPP__
#define COMMAND_GROUP_HPP__

#pragma once

#include <cstdint>

#include "motor_command.hpp"

// Command Groups
namespace CommandGroups 
{
  constexpr uint8_t GET_STATUS_CMDS[] = {
    GET_CURRENT, GET_TARGET_CURRENT,
    GET_VELOCITY, GET_TARGET_VELOCITY,
    GET_POSITION, GET_TARGET_POSITION,
  };

  constexpr uint8_t GET_PID_CMDS[] = {
    GET_VELOCITY_P, GET_VELOCITY_I, 
    GET_POSITION_P,                 GET_POSITION_D,
    // GET_CURRENT_P, GET_CURRENT_I, GET_CURRENT_D
  };

  constexpr uint8_t GET_LIMIT_CMDS[] = {
    GET_MAX_CURRENT, 
    GET_MAX_POS_CURRENT, GET_MAX_NEG_CURRENT,
    GET_MAX_ACCEL, GET_MIN_ACCEL,
    GET_MAX_VELOCITY, GET_MIN_VELOCITY,
    GET_MAX_POSITION, GET_MIN_POSITION,
    GET_OVERVOLT_THRESH, GET_UNDERVOLT_THRESH,
    GET_MOTOR_OT_THRESH, GET_BOARD_OT_THRESH
  };

  constexpr uint8_t GET_CONFIGS[] = {
    GET_MODE,
    GET_POSITION_OFFSET,
    GET_ERROR_STATUS, 
    GET_BUS_VOLTAGE,
    GET_MOTOR_TEMP, GET_BOARD_TEMP,
    GET_ENCODER_VOLTAGE, GET_ENCODER_STATUS
  };

} // namespace CommandGroups

#endif // COMMAND_GROUP_HPP__