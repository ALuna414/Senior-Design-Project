#include "RotationPID.h"
#include "Gyro.h"
#include <arduino.h>

RotationPID::RotationPID(float P, float I, float D)
{
  setPID(P, I, D);
  setPIDRate(P, I, D);
//  for(int i = 0; i < ACCUMULATOR_LENGTH; i++)
//    rolling_errors[i] = 0;
}

float RotationPID::update(float dt, Gyro* gyro)
{  

  // float error = wrapError(target_angle - gyro->angle);
  // Serial.print("ERROR IN PID UPDATE: "); Serial.println(error);
  // return P * error;
  float error = 0;
  if(mode == hold_angle)
  {
    // float angle_remaining = wrapError(target_angle - virtual_target);
    
    // if(mag(angle_remaining) < turn_rate * dt)
    // {
    //   virtual_target = target_angle;
    // }
    // else
    // {
    //   if(angle_remaining > 0)
    //     virtual_target += turn_rate * dt;
    //   else
    //     virtual_target -= turn_rate * dt;
    // }
    error = wrapError(target_angle - gyro->angle);//-wrapError(gyro->angle - virtual_target);
  }
  
  else if(mode == hold_rate)
  {
    error = target_angle - gyro->rate;
  }
  
  float error_rate = (error - prev_error) / dt;
  prev_error = error;

  error_sum += error * dt;
  if(error_sum > integral_cap)
    error_sum = integral_cap;
  else if(error_sum < -integral_cap)
    error_sum = -integral_cap;
  
  if(mode == hold_angle)
  {
    return P * error + I * error_sum + D * error_rate;
  }
  return P_rate * error + I_rate * error_sum + D_rate * error_rate;
  
}
void RotationPID::setMode(Mode new_mode)
{
  if(new_mode == mode)
    return;
//  for(int i = 0; i < ACCUMULATOR_LENGTH; i++)
//    rolling_errors[i] = 0;
  prev_error = 0;
  error_sum = 0;

  mode = new_mode;
}
void RotationPID::setTargetAngle(float angle)
{
  if(mode == hold_angle)
    angle = wrap360(angle);
  this->target_angle = angle;
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
void RotationPID::setPIDRate(float P, float I, float D)
{
  this->P_rate = P;
  this->I_rate = I;
  this->D_rate = D;
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
