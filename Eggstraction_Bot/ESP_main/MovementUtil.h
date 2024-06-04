#ifndef MOVEMENT_UTIL
#define MOVEMENT_UTIL

#include "ESP2UNOComm.h"
#include "Gyro.h"
#include "PID.h"

class MovementUtil
{
private:
  ESP2UNOComm* UNO;
  Gyro* gyro;
  PID rotPID = PID(0.015, 0.01, 0, 0.1);
  PID trackPID = PID(0.015, 0.01, 0, 0.1);
  PID ratePID = PID(0.008, 0.005, 0, 0.1);
  PID headingPID = PID(5, 0, 0);

  float hold_rot_target = 0;
  
public:
  MovementUtil(){}
  MovementUtil(ESP2UNOComm* UNO, Gyro* gyro) : UNO(UNO), gyro(gyro){}
  
  void scan(float search_spd, float dt);
  bool seek(float target_heading, float target_dist, float dt);
  float track(float target_heading, float dt);
  void setForward(float target_speed, float dt);
  void setRotTarget(float angle);
  void setRotTarget();
  void holdHeading(float dt);
  void holdHeading(float target_heading, float dt);
  void cutMotors();
  void goToHeading(float target_heading, float max_rate, float dt);
};

#endif
