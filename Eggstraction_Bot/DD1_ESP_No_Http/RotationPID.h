#ifndef ROTATION_PID
#define ROTATION_PID


#define ACCUMULATOR_LENGTH 50

class RotationPID
{
private:
  float P = 0.02;
  float I = 0;
  float D = 0;

  float deadzone = 0;

  float wrap360(float num); // constrain an angular value to [0, 360) degrees
  float wrapError(float error);
  float mag(float num);

  float prev_error = 0;
  
  float rolling_errors[ACCUMULATOR_LENGTH];
  int err_pos = 0;
  

public:
  float accumulator = 0;

  
  float virtual_target = 0;
  float target_angle = 0;
  float turn_rate = 180;
  
  RotationPID(){}
  RotationPID(float P, float I, float D, float deadzone = 0);

  float update(float dt, float heading);
  float getErrorABS();
  void setTargetAngle(float angle);
  void setMaxRate(float rate);
  void setPID(float P, float I, float D);
};

#endif
