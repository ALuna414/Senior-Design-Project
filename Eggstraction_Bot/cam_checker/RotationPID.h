#ifndef ROTATION_PID
#define ROTATION_PID


//#define ACCUMULATOR_LENGTH 50

class Gyro;

class RotationPID
{
private:
  float P = 0.02;
  float I = 0;
  float D = 0;

  float P_rate = 0;
  float I_rate = 0;
  float D_rate = 0;

  float wrap360(float num); // constrain an angular value to [0, 360) degrees
  float wrapError(float error);
  float mag(float num);

  float prev_error = 0;
  float error_sum = 0;
  float integral_cap = 90;
  

public:

  enum Mode
  {
    hold_angle,
    hold_rate
  };
  Mode mode = hold_angle;

  void setMode(Mode new_mode);
  float accumulator = 0;

  
  float virtual_target = 0;
  float target_angle = 0;
  float turn_rate = 180;
  
  RotationPID(){}
  RotationPID(float P, float I, float D);

  float update(float dt, Gyro* gyro);
  float getErrorABS();
  void setTargetAngle(float angle);
  void setMaxRate(float rate);
  void setPID(float P, float I, float D);
  void setPIDRate(float P, float I, float D);
};

#endif
