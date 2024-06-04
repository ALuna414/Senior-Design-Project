#include "MovementUtil.h"

void MovementUtil::scan(float search_spd, float dt)
{  
  setForward(0, dt);  
  UNO->setMotorsAngular(ratePID.update(search_spd - gyro->rate, dt));
}

bool MovementUtil::seek(float target_heading, float target_dist, float dt)
{    
  float error = track(target_heading, dt);

  bool at_target = false;
  float speed = 0;
  
  if(target_dist < 21)
    speed = 0.6;
  else if(target_dist < 46)
    speed = 0.1;
  else
    at_target = true;
  
  setForward(speed, dt);
  return at_target;
}

float MovementUtil::track(float target_heading, float dt)
{
  float error = gyro->relativeAngle(target_heading);
  UNO->setMotorsAngular(trackPID.update(error, dt));
  return error;
}

void MovementUtil::setForward(float target_speed, float dt)
{
  const float linear_accel = 0.5;
  float speed = UNO->TX_data.motor_control_linear;
  
  if(target_speed - speed <= linear_accel * dt)
    UNO->setMotorsLinear(target_speed);
  else
    UNO->setMotorsLinear(speed + linear_accel * dt);
}

void MovementUtil::cutMotors()
{
  UNO->setMotorsAngular(0);
  UNO->setMotorsLinear(0);
  rotPID.clear();
  ratePID.clear();
  trackPID.clear();
}

void MovementUtil::holdHeading(float dt)
{
  holdHeading(hold_rot_target, dt);
}

void MovementUtil::holdHeading(float target_heading, float dt)
{
  float error = gyro->relativeAngle(gyro->angle(), target_heading);
  UNO->setMotorsAngular(rotPID.update(error, dt));
}

void MovementUtil::setRotTarget(float angle)
{
  hold_rot_target = angle;
  rotPID.clear();
}

void MovementUtil::setRotTarget()
{
  setRotTarget(gyro->angle());
}

void MovementUtil::goToHeading(float target_heading, float max_rate, float dt)
{
  float rate = headingPID.update(gyro->relativeAngle(target_heading), dt);
  if(rate > max_rate)
    rate = max_rate;
  else if(rate < -max_rate)
    rate = -max_rate;

  UNO->setMotorsAngular(ratePID.update(rate - gyro->rate, dt));
}
