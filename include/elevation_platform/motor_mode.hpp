#ifndef MOTOR_MODE_HPP__
#define MOTOR_MODE_HPP__

#pragma once

#include <cstdint>

enum MotorMode : uint8_t 
{
  STOP = 0,
  CURRENT = 1,
  VELOCITY = 2,
  POSITION = 3
};

#endif // MOTOR_MODE_HPP__