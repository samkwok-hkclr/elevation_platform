#ifndef MOTOR_COMMAND_HPP__
#define MOTOR_COMMAND_HPP__

#pragma once

#include <cstdint>

enum MotorCommand : uint8_t 
{
  // Basic Control Commands
  CLEAR_ERRORS          = 0x0B,
  RESTORE_FLASH         = 0x0D,
  SAVE_TO_FLASH         = 0x0E,
  FACTORY_RESET         = 0x0F,
  SAVE_AS_FACTORY       = 0x1F,
  
  // Status Monitoring Commands
  GET_MODE              = 0x03,
  GET_CURRENT           = 0x04,
  GET_TARGET_CURRENT    = 0x05,
  GET_VELOCITY          = 0x06,
  GET_TARGET_VELOCITY   = 0x07,
  GET_POSITION          = 0x08,
  GET_TARGET_POSITION   = 0x09,
  GET_ERROR_STATUS      = 0x0A,
  GET_BUS_VOLTAGE       = 0x14,
  GET_MOTOR_TEMP        = 0x31,
  GET_BOARD_TEMP        = 0x32,
  GET_ENCODER_VOLTAGE   = 0x78,
  GET_ENCODER_STATUS    = 0x79,

  // PID Control Commands
  GET_VELOCITY_P        = 0x10,
  GET_VELOCITY_I        = 0x11,
  GET_VELOCITY_D        = 0x33,
  GET_POSITION_P        = 0x12,
  GET_POSITION_I        = 0x34,
  GET_POSITION_D        = 0x13,

  // Limit Settings Commands
  GET_MAX_CURRENT       = 0x37,
  GET_MAX_POS_CURRENT   = 0x35,
  GET_MAX_NEG_CURRENT   = 0x36,
  GET_MAX_ACCEL         = 0x16,
  GET_MIN_ACCEL         = 0x17,
  GET_MAX_VELOCITY      = 0x18,
  GET_MIN_VELOCITY      = 0x19,
  GET_MAX_POSITION      = 0x1A,
  GET_MIN_POSITION      = 0x1B,
  GET_OVERVOLT_THRESH   = 0x8A,
  GET_UNDERVOLT_THRESH  = 0x8C,
  GET_MOTOR_OT_THRESH   = 0x8F,
  GET_BOARD_OT_THRESH   = 0x93,

  // Motor Identification Commands
  GET_MOTOR_MODEL       = 0x64,
  GET_MOTOR_VERSION     = 0x65,
  GET_SW_VERSION        = 0x66,

  // Encoder Commands
  GET_POSITION_OFFSET   = 0x54,
  GET_CSP_DATA          = 0x41,

  // Mode Control Commands
  SET_CURRENT_MODE     = 0x1C,
  SET_VELOCITY_MODE    = 0x1D,
  SET_POSITION_MODE    = 0x1E,
  MOTOR_STOP           = 0x02,

  // Combined Set+Get Commands
  SET_CURRENT_GET_CSP  = 0x42,
  SET_VELOCITY_GET_CSP = 0x43,
  SET_POSITION_GET_CSP = 0x44,

  // Limit Configuration
  SET_MAX_POS_CURRENT  = 0x20,
  SET_MIN_NEG_CURRENT  = 0x21,
  SET_MAX_ACCEL        = 0x22,
  SET_MIN_ACCEL        = 0x23,
  SET_MAX_VELOCITY     = 0x24,
  SET_MIN_VELOCITY     = 0x25,
  SET_MAX_POSITION     = 0x26,
  SET_MIN_POSITION     = 0x27,

  // PID Configuration
  SET_POSITION_P       = 0x2B,
  SET_POSITION_I       = 0x2C,
  SET_POSITION_D       = 0x2D,
  SET_VELOCITY_P       = 0x29,
  SET_VELOCITY_I       = 0x2A,
  SET_VELOCITY_D       = 0x2B,

  // System Configuration
  SET_CAN_ID           = 0x2E,
  SET_BAUDRATE         = 0x3F,
  ENCODER_ZERO         = 0x50,
  SET_POSITION_OFFSET  = 0x53,

  // Safety Thresholds
  SET_OVERVOLT_THRESH  = 0x87,
  SET_UNDERVOLT_THRESH = 0x89,
  SET_MOTOR_OT_THRESH  = 0x8D,
  SET_BOARD_OT_THRESH  = 0x91,

  // Special Modes
  SET_LIMIT_FLAGS      = 0x55,
  SET_COMM_TIMEOUTS    = 0x59,
  SET_FEEDFORWARD      = 0x47,

};

#endif // MOTOR_COMMAND_HPP__