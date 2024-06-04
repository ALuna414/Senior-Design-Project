#ifndef PID_
#define PID_

class PID
{
private:
  float P, I, D;
  float epsilon = 0; // accumulator won't exceed (error / epsilon) in steady state (e = 0 yields normal I response)

  float prev_error = 0;
  float accum = 0;

public:
  PID(){}
  PID(float P, float I, float D, float e = 0) : P(P), I(I), D(D), epsilon(e){}

  float update(float error, float dt);
  void clear();
};

#endif
