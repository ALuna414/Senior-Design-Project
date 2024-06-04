#include "RotationPID.h"
#include <arduino.h>

RotationPID::RotationPID(float P, float I, float D, float deadzone)
{
  setPID(P, I, D);
  this->deadzone = deadzone;
  for(int i = 0; i < ACCUMULATOR_LENGTH; i++)
    rolling_errors[i] = 0;
}

float RotationPID::update(float dt, float heading)
{  
  float angle_remaining = wrapError(target_angle - virtual_target);
  
  if(mag(angle_remaining) < turn_rate * dt)
  {
    virtual_target = target_angle;
  }
  else
  {
    if(angle_remaining > 0)
      virtual_target += turn_rate * dt;
    else
      virtual_target -= turn_rate * dt;
  }

  float error = -wrapError(heading - virtual_target);

  if(error > 0 && error < deadzone) error = 0;
  else if(error < 0 && error > -deadzone) error = 0;
  
  float error_rate = (error - prev_error) / dt;
  prev_error = error;

  rolling_errors[err_pos] = error * dt;    // store new error integral
  accumulator += rolling_errors[err_pos];  // accumulate new error integral
  err_pos = (err_pos + 1) % ACCUMULATOR_LENGTH; // move to next pos (the oldest value)
  accumulator -= rolling_errors[err_pos];        // un-accumulate the oldest value
  
  return P * error + I * accumulator + D * error_rate;
}
void RotationPID::setTargetAngle(float angle)
{
  this->target_angle = wrap360(angle);
}
void RotationPID::setMaxRate(float rate)
{
  if(rate < 0)
    rate = -rate;
  this->turn_rate = rate;
}
void RotationPID::setPID(float P, float I, float D)
{
  this->P = P;
  this->I = I;
  this->D = D;
}

float RotationPID::getErrorABS()
{
  if(prev_error < 0)
    return -prev_error;
  return prev_error;
}


float RotationPID::wrap360(float num)
{
  // Possible but unlikely long loop times here, but I'm pretty sure an
  // infinite loop is impossible.
  while(num > 360)
    num -= 360;
  while(num < 0)
    num += 360;

  return num;
}

float RotationPID::wrapError(float error)
{
  if(error > 180)
    error = -(360 - error);
  else if(error < -180)
    error = 360 + error;
  return error;
}

float RotationPID::mag(float num)
{
  if(num < 0)
    return -num;
  return num;
}
