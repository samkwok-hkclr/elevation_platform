#ifndef MOTOR_STATUS_HPP__
#define MOTOR_STATUS_HPP__

#pragma once

#include <cstring>

class MotorStatus 
{
private:
  uint8_t operation_mode_;

  float current_;
  float target_current_;
  float velocity_;
  float target_velocity_;
  uint32_t position_;
  uint32_t target_position_;

  uint32_t position_offset_;
  
  // Error information
  uint32_t error_status_;
  
  // PID control parameters
  struct PidParams 
  {
    float kp_;
    float ki_;
    float kd_;
  } velocity_loop_, position_loop_;
  
  // Power information
  float bus_voltage_;
  float motor_temp_;
  float board_temp_;
  
  // Current limits
  float max_current_abs_;
  float max_forward_current_;
  float min_backward_current_;
  
  // Acceleration limits
  float max_forward_accel_;
  float min_backward_accel_;
  
  // Velocity limits
  float max_forward_velocity_;
  float min_backward_velocity_;
  
  // Position limits
  float max_forward_position_;
  float min_backward_position_;

public:
  // Constructor
  MotorStatus() 
  {
    memset(this, 0, sizeof(MotorStatus));
  }

  // Getters
  uint8_t operation_mode() const { return operation_mode_; }
  
  float current() const { return current_; }
  float target_current() const { return target_current_; }
  float velocity() const { return velocity_; }
  float target_velocity() const { return target_velocity_; }
  uint32_t position() const { return position_; }
  uint32_t target_position() const { return target_position_; }
  uint32_t position_offset() const { return position_offset_; }
  uint32_t error_status() const { return error_status_; }
  float bus_voltage() const { return bus_voltage_; }
  float motor_temp() const { return motor_temp_; }
  float board_temp() const { return board_temp_; }
  float max_current_abs() const { return max_current_abs_; }
  float max_forward_current() const { return max_forward_current_; }
  float min_backward_current() const { return min_backward_current_; }
  float max_forward_accel() const { return max_forward_accel_; }
  float min_backward_accel() const { return min_backward_accel_; }
  float max_forward_velocity() const { return max_forward_velocity_; }
  float min_backward_velocity() const { return min_backward_velocity_; }
  float max_forward_position() const { return max_forward_position_; }
  float min_backward_position() const { return min_backward_position_; }

  // PID getters
  float velocity_loop_kp() const { return velocity_loop_.kp_; }
  float velocity_loop_ki() const { return velocity_loop_.ki_; }
  float velocity_loop_kd() const { return velocity_loop_.kd_; }
  float position_loop_kp() const { return position_loop_.kp_; }
  float position_loop_ki() const { return position_loop_.ki_; }
  float position_loop_kd() const { return position_loop_.kd_; }

  // Setters
  void set_operation_mode(int mode) { operation_mode_ = mode; }
  void set_current(float current) { current_ = current; }
  void set_target_current(float target) { target_current_ = target; }
  void set_velocity(float velocity) { velocity_ = velocity; }
  void set_target_velocity(uint32_t target) { target_velocity_ = target; }
  void set_position(uint32_t pos) { position_ = pos; }
  void set_position_offset(uint32_t pos) { position_offset_ = pos; }
  void set_target_position(float target) { target_position_ = target; }
  void set_error_status(uint32_t status) { error_status_ = status; }
  void set_bus_voltage(float voltage) { bus_voltage_ = voltage; }
  void set_motor_temp(float temp) { motor_temp_ = temp; }
  void set_board_temp(float temp) { board_temp_ = temp; }
  void set_max_current_abs(float current) { max_current_abs_ = current; }
  void set_max_forward_current(float current) { max_forward_current_ = current; }
  void set_min_backward_current(float current) { min_backward_current_ = current; }
  void set_max_forward_accel(float accel) { max_forward_accel_ = accel; }
  void set_min_backward_accel(float accel) { min_backward_accel_ = accel; }
  void set_max_forward_velocity(float velocity) { max_forward_velocity_ = velocity; }
  void set_min_backward_velocity(float velocity) { min_backward_velocity_ = velocity; }
  void set_max_forward_position(float pos) { max_forward_position_ = pos; }
  void set_min_backward_position(float pos) { min_backward_position_ = pos; }

  // PID setters
  void set_velocity_loop_params(float kp, float ki, float kd) 
  {
    velocity_loop_.kp_ = kp;
    velocity_loop_.ki_ = ki;
    velocity_loop_.kd_ = kd;
  }
  void set_position_loop_params(float kp, float ki, float kd) 
  {
    position_loop_.kp_ = kp;
    position_loop_.ki_ = ki;
    position_loop_.kd_ = kd;
  }

  // Utility methods
  bool has_errors() const 
  {
    return error_status_ != 0;
  }

  // Reset all values to zero
  void reset() {
    memset(this, 0, sizeof(MotorStatus));
  }
};

#endif