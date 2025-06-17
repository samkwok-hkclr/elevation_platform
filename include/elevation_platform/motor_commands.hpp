#ifndef MOTOR_COMMANDS_HPP__
#define MOTOR_COMMANDS_HPP__

#pragma once

#include <cstdint>

namespace MotorCommands {
  // Basic Control Commands
  constexpr uint8_t CLEAR_ERRORS          = 0x0B;
  constexpr uint8_t RESTORE_FLASH         = 0x0D;
  constexpr uint8_t SAVE_TO_FLASH         = 0x0E;
  constexpr uint8_t FACTORY_RESET         = 0x0F;
  constexpr uint8_t SAVE_AS_FACTORY       = 0x1F;
  
  // Status Monitoring Commands
  constexpr uint8_t GET_MODE              = 0x03;
  constexpr uint8_t GET_CURRENT           = 0x04;
  constexpr uint8_t GET_TARGET_CURRENT    = 0x05;
  constexpr uint8_t GET_VELOCITY          = 0x06;
  constexpr uint8_t GET_TARGET_VELOCITY   = 0x07;
  constexpr uint8_t GET_POSITION          = 0x08;
  constexpr uint8_t GET_TARGET_POSITION   = 0x09;
  constexpr uint8_t GET_ERROR_STATUS      = 0x0A;
  constexpr uint8_t GET_BUS_VOLTAGE       = 0x14;
  constexpr uint8_t GET_MOTOR_TEMP        = 0x31;
  constexpr uint8_t GET_BOARD_TEMP        = 0x32;
  constexpr uint8_t GET_ENCODER_VOLTAGE   = 0x78;
  constexpr uint8_t GET_ENCODER_STATUS    = 0x79;

  // PID Control Commands
  constexpr uint8_t GET_VELOCITY_P        = 0x10;
  constexpr uint8_t GET_VELOCITY_I        = 0x11;
  constexpr uint8_t GET_VELOCITY_D        = 0x33;
  constexpr uint8_t GET_POSITION_P        = 0x12;
  constexpr uint8_t GET_POSITION_I        = 0x34;
  constexpr uint8_t GET_POSITION_D        = 0x13;
  constexpr uint8_t GET_CURRENT_P         = 0x61;
  constexpr uint8_t GET_CURRENT_I         = 0x62;
  constexpr uint8_t GET_CURRENT_D         = 0x63;

  // Limit Settings Commands
  constexpr uint8_t GET_MAX_CURRENT       = 0x37;
  constexpr uint8_t GET_MAX_POS_CURRENT   = 0x35;
  constexpr uint8_t GET_MAX_NEG_CURRENT   = 0x36;
  constexpr uint8_t GET_MAX_ACCEL         = 0x16;
  constexpr uint8_t GET_MIN_ACCEL         = 0x17;
  constexpr uint8_t GET_MAX_VELOCITY      = 0x18;
  constexpr uint8_t GET_MIN_VELOCITY      = 0x19;
  constexpr uint8_t GET_MAX_POSITION      = 0x1A;
  constexpr uint8_t GET_MIN_POSITION      = 0x1B;
  constexpr uint8_t GET_OVERVOLT_THRESH   = 0x8A;
  constexpr uint8_t GET_UNDERVOLT_THRESH  = 0x8C;
  constexpr uint8_t GET_MOTOR_OT_THRESH   = 0x8F;
  constexpr uint8_t GET_BOARD_OT_THRESH   = 0x93;

  // Motor Identification Commands
  constexpr uint8_t GET_MOTOR_MODEL       = 0x64;
  constexpr uint8_t GET_MOTOR_VERSION     = 0x65;
  constexpr uint8_t GET_SW_VERSION        = 0x66;

  // Encoder Commands
  constexpr uint8_t GET_POSITION_OFFSET   = 0x54;
  constexpr uint8_t GET_CSP_DATA          = 0x41;

  // Mode Control Commands
  constexpr uint8_t SET_CURRENT_MODE     = 0x1C;
  constexpr uint8_t SET_VELOCITY_MODE    = 0x1D;
  constexpr uint8_t SET_POSITION_MODE    = 0x1E;
  constexpr uint8_t MOTOR_STOP           = 0x02;

  // Combined Set+Get Commands
  constexpr uint8_t SET_CURRENT_GET_CSP  = 0x42;
  constexpr uint8_t SET_VELOCITY_GET_CSP = 0x43;
  constexpr uint8_t SET_POSITION_GET_CSP = 0x44;

  // Limit Configuration
  constexpr uint8_t SET_MAX_POS_CURRENT  = 0x20;
  constexpr uint8_t SET_MIN_NEG_CURRENT  = 0x21;
  constexpr uint8_t SET_MAX_ACCEL        = 0x22;
  constexpr uint8_t SET_MIN_ACCEL        = 0x23;
  constexpr uint8_t SET_MAX_VELOCITY     = 0x24;
  constexpr uint8_t SET_MIN_VELOCITY     = 0x25;
  constexpr uint8_t SET_MAX_POSITION     = 0x26;
  constexpr uint8_t SET_MIN_POSITION     = 0x27;

  // PID Configuration
  constexpr uint8_t SET_VELOCITY_P       = 0x29;
  constexpr uint8_t SET_VELOCITY_I       = 0x2A;
  constexpr uint8_t SET_VELOCITY_D       = 0x2B;
  constexpr uint8_t SET_POSITION_P       = 0x2B;
  constexpr uint8_t SET_POSITION_I       = 0x2C;
  constexpr uint8_t SET_POSITION_D       = 0x2D;

  // System Configuration
  constexpr uint8_t SET_CAN_ID           = 0x2E;
  constexpr uint8_t SET_BAUDRATE         = 0x3F;
  constexpr uint8_t ENCODER_ZERO         = 0x50;
  constexpr uint8_t SET_POSITION_OFFSET  = 0x53;

  // Safety Thresholds
  constexpr uint8_t SET_OVERVOLT_THRESH  = 0x87;
  constexpr uint8_t SET_UNDERVOLT_THRESH = 0x89;
  constexpr uint8_t SET_MOTOR_OT_THRESH  = 0x8D;
  constexpr uint8_t SET_BOARD_OT_THRESH  = 0x91;

  // Special Modes
  constexpr uint8_t SET_LIMIT_FLAGS      = 0x55;
  constexpr uint8_t SET_COMM_TIMEOUTS    = 0x59;
  constexpr uint8_t SET_FEEDFORWARD      = 0x47;

  // Command Groups
  namespace CommandGroups 
  {
    constexpr uint8_t STATUS_COMMANDS[] = {
      GET_MODE, GET_CURRENT, GET_TARGET_CURRENT,
      GET_VELOCITY, GET_TARGET_VELOCITY,
      GET_POSITION, GET_TARGET_POSITION,
      GET_ERROR_STATUS, GET_BUS_VOLTAGE,
      GET_MOTOR_TEMP, GET_BOARD_TEMP
    };

    constexpr uint8_t PID_COMMANDS[] = {
      GET_VELOCITY_P, GET_VELOCITY_I, GET_VELOCITY_D,
      GET_POSITION_P, GET_POSITION_I, GET_POSITION_D,
      GET_CURRENT_P, GET_CURRENT_I, GET_CURRENT_D
    };

    constexpr uint8_t LIMIT_COMMANDS[] = {
      GET_MAX_CURRENT, GET_MAX_POS_CURRENT, GET_MAX_NEG_CURRENT,
      GET_MAX_ACCEL, GET_MIN_ACCEL,
      GET_MAX_VELOCITY, GET_MIN_VELOCITY,
      GET_MAX_POSITION, GET_MIN_POSITION,
      GET_OVERVOLT_THRESH, GET_UNDERVOLT_THRESH,
      GET_MOTOR_OT_THRESH, GET_BOARD_OT_THRESH
    };
  } // namespace CommandGroups

} // namespace MotorCommands

#endif // MOTOR_COMMANDS_HPP__