#include "RotationPID.h"
#include <arduino.h>

RotationPID::RotationPID(float P, float I, float D)
{
  setPID(P, I, D);
}

float RotationPID::update(float dt, float heading)
{  
  float angle_remaining = wrapError(target_angle - virtual_target);
  
  if(mag(angle_remaining) < turn_rate * dt)
  {
//    Serial.print("mag angle remaining: "); Serial.print(mag(angle_remaining));
//    Serial.print("\tturn rate: "); Serial.println(turn_rate);
//    Serial.print("\tdt: "); Serial.println(dt);
//    Serial.print("\tturn rate * dt: "); Serial.println(turn_rate * dt);
//    Serial.println("SET TARGETS EQUAL");
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
  
  Serial.print("\ttarget: "); Serial.print(target_angle);
  Serial.print("\tvirt target: "); Serial.print(virtual_target);
  Serial.print("error: "); Serial.println(error);
  
  float error_rate = (error - prev_error) / dt;
  prev_error = error;

  accumulator += error * dt;

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
