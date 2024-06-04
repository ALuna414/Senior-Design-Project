#ifndef ROTATION_PID
#define ROTATION_PID

class RotationPID
{
private:
  float P = 0.02;
  float I = 0;
  float D = 0;

  float wrap360(float num); // constrain an angular value to [0, 360) degrees
  float wrapError(float error);
  float mag(float num);

  float prev_error = 0;
  float accumulator = 0;
  

public:
  float virtual_target = 0;
  float target_angle = 0;
  float turn_rate = 180;
  
  RotationPID(){}
  RotationPID(float P, float I, float D);

  float update(float dt, float heading);
  float getErrorABS();
  void setTargetAngle(float angle);
  void setMaxRate(float rate);
  void setPID(float P, float I, float D);
};

#endif
