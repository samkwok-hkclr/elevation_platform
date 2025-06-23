#ifndef MOTOR_STATUS_HPP__
#define MOTOR_STATUS_HPP__

#pragma once

#include <cstring>
#include <atomic>

class MotorStatus 
{
public:
  MotorStatus() 
  {
    memset(this, 0, sizeof(MotorStatus));
  }

  std::atomic<uint8_t> operation_mode{0};

  std::atomic<float> current_degree{0.0};

  std::atomic<float> current{0.0};
  std::atomic<float> target_current{0.0};
  std::atomic<float> velocity{0.0};
  std::atomic<float> target_velocity{0.0};
  std::atomic<uint32_t> position{0};
  std::atomic<uint32_t> target_position{0};

  std::atomic<uint32_t> position_offset{0};
  
  // Error information
  std::atomic<uint32_t> error_status{0};
  
  // PID control parameters
  std::atomic<float> position_p{0.0};
  std::atomic<float> position_i{0.0};
  std::atomic<float> position_d{0.0}; 
  std::atomic<float> velocity_p{0.0};
  std::atomic<float> velocity_i{0.0};
  std::atomic<float> velocity_d{0.0};
  std::atomic<float> current_p{0.0};
  std::atomic<float> current_i{0.0};
  std::atomic<float> current_d{0.0}; 

  // Power information
  std::atomic<float> bus_voltage{0.0};
  std::atomic<float> encoder_voltage{0.0};
  std::atomic<float> encoder_status{0.0};
  std::atomic<float> motor_temp{0.0};
  std::atomic<float> board_temp{0.0};
  
  // Current limits
  std::atomic<float> max_current_abs{0.0};
  std::atomic<float> max_forward_current{0.0};
  std::atomic<float> min_backward_current{0.0};
  
  // Acceleration limits
  std::atomic<float> max_forward_accel{0.0};
  std::atomic<float> min_backward_accel{0.0};
  
  // Velocity limits
  std::atomic<float> max_forward_velocity{0.0};
  std::atomic<float> min_backward_velocity{0.0};
  
  // Position limits
  std::atomic<uint32_t> max_forward_position{0};
  std::atomic<uint32_t> min_backward_position{0};

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