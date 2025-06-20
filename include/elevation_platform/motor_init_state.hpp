#ifndef MOTOR_INIT_STATE_HPP__
#define MOTOR_INIT_STATE_HPP__

#include <atomic>
#include <mutex>

class MotorInitState 
{
public:
  MotorInitState() = default;
  ~MotorInitState() = default;

  // Position offset state
  bool is_pos_offset_initialized() const 
  { 
    return is_pos_offset_.load(); 
  }
  void set_pos_offset_initialized(bool state) 
  {
    is_pos_offset_.store(state); 
  }

  // Max current state
  // bool is_max_current_initialized() const 
  // { 
  //   return is_max_current_.load(); 
  // }
  // void set_max_current_initialized(bool state) 
  // { 
  //   is_max_current_.store(state); 
  // }

  // Min current state
  // bool is_min_current_initialized() const 
  // {
  //   return is_min_current_.load(); 
  // }
  // void set_min_current_initialized(bool state) 
  // { 
  //   is_min_current_.store(state); 
  // }

  // Check if all parameters are initialized
  bool all_initialized() const 
  {
    // return is_pos_offset_.load() && is_max_current_.load() && is_min_current_.load();
    return is_pos_offset_.load();
  }

private:
  std::atomic<bool> is_pos_offset_{false};
  // std::atomic<bool> is_max_current_{false};
  // std::atomic<bool> is_min_current_{false};
};

#endif // MOTOR_INIT_STATE_HPP__