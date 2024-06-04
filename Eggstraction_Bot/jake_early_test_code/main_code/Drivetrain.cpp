#include "Drivetrain.h"

Drivetrain::Drivetrain(int l_a, int l_b, int r_a, int r_b)
{
  forward[0] = l_a;
  backward[0] = l_b;
  forward[1] = r_a;
  backward[1] = r_b;
}

void Drivetrain::begin()
{
  pinMode(forward[0], OUTPUT);
  pinMode(backward[0], OUTPUT);
  pinMode(forward[1], OUTPUT);
  pinMode(backward[1], OUTPUT);

  digitalWrite(forward[0], 0);
  digitalWrite(backward[0], 0);
  digitalWrite(forward[1], 0);
  digitalWrite(backward[1], 0);
}

// val from -1 to 1;
void Drivetrain::setSpeed(float val)
{
  fwd_spd = bound(val);

  setMotors();
}

// val from -1 to 1
void Drivetrain::setTurnRate(float val)
{
  turn_rate = bound(val);

  setMotors();
}

// 0 for left, 1 for right
void Drivetrain::setMotors()
{
  int pwm_vals[2];
  pwm_vals[0] = throttleMap(fwd_spd - turn_rate);
  pwm_vals[1] = throttleMap(fwd_spd + turn_rate);
  for(int motor = 0; motor < 2; motor++)
  {
    if(pwm_vals[motor] < 0)
    {
      digitalWrite(backward[motor], 0);
      analogWrite(forward[motor], -pwm_vals[motor]);
    }
    else if(pwm_vals[motor] > 0)
    {
      digitalWrite(forward[motor], 0);
      analogWrite(backward[motor], pwm_vals[motor]);
    }
    else
    {
      digitalWrite(forward[motor], 0);
      digitalWrite(backward[motor], 0);
    }
  }
}

float Drivetrain::bound(float val, float bound_radius)
{
  if(val > bound_radius)
    val = bound_radius;
  else if(val < -bound_radius)
    val = -bound_radius;
  return val;
}

float Drivetrain::throttleMap(float val)
{
  const float y_int = 0.15; // y_intercept
  val = bound(val);
  if(val > 0)
  {
    val = (1 - y_int) * val + y_int;
  }
  else if(val < 0)
  {
    val = (1 - y_int) * val - y_int;
  }

  return val * 255;
}
