#ifndef IR_SENSOR
#define IR_SENSOR

#include <Arduino.h>

class IRSensor
{
private:
  int analog_pin;
  
public:
  IRSensor(){}
  IRSensor(int analog_pin);

  void setAnalogPin(int pin);
  void begin();

  float read();
  int readRaw();
};

#endif
