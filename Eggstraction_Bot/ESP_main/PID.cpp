#include "PID.h"

float PID::update(float error, float dt)
{
  float rate = (error - prev_error) / dt;
  prev_error = error;
  
  accum *= (1 - epsilon * dt);  // optional exponential decay
  accum += error * dt;

  return P * error + I * accum + D * rate;
}

void PID::clear()
{
  prev_error = 0;
  accum = 0;
}
