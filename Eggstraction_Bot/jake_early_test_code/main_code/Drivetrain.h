#ifndef DRIVETRAIN_
#define DRIVETRAIN_

#include <Arduino.h>

class Drivetrain
{
private:  // stuff the object uses: you can't use these functions and variables in the rest of the program
  // 0 for left, 1 for right
  uint8_t forward[2];   // the two 'A' pins
  uint8_t backward[2];  // the two 'B' pins

  float throttleMap(float val); // maps lower throttle inputs to higher outputs, since motor stalls at PWM below about 20%
  void setMotors();
  float fwd_spd = 0;
  float turn_rate = 0;
  
public: // these are the things you should use in the client program
  Drivetrain(){}
  Drivetrain(int l_a, int l_b, int r_a, int r_b);

  void begin(); // initialize pins

  void setSpeed(float val); // unitless speed value from -1 to 1
  void setTurnRate(float val); // unitless angular rate from -1 to 1
  

  float bound(float val, float bound_radius = 1); // returns the input bounded to [-bound_radius, bound_radius]
  
};

#endif
