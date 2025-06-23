#ifndef MOTOR_STATUS_HPP__
#define MOTOR_STATUS_HPP__

#pragma once

#include <cstring>

class MotorStatus 
{
public:
  MotorStatus() 
  {
    memset(this, 0, sizeof(MotorStatus));
  }

  uint8_t operation_mode;

  float current_degree;

  float current;
  float target_current;
  float velocity;
  float target_velocity;
  uint32_t position;
  uint32_t target_position;

  uint32_t position_offset;
  
  // Error information
  uint32_t error_status;
  
  // PID control parameters
  float position_p;
  float position_i;
  float position_d; 
  float velocity_p;
  float velocity_i;
  float velocity_d;
  float current_p;
  float current_i;
  float current_d; 

  // Power information
  float bus_voltage;
  float encoder_voltage;
  float encoder_status;
  float motor_temp;
  float board_temp;
  
  // Current limits
  float max_current_abs;
  float max_forward_current;
  float min_backward_current;
  
  // Acceleration limits
  float max_forward_accel;
  float min_backward_accel;
  
  // Velocity limits
  float max_forward_velocity;
  float min_backward_velocity;
  
  // Position limits
  uint32_t max_forward_position;
  uint32_t min_backward_position;

  // Utility methods
  bool has_errors() const 
  {
    return error_status != 0;
  }

  // Reset all values to zero
  void reset() 
  {
    memset(this, 0, sizeof(MotorStatus));
  }
};

#endif