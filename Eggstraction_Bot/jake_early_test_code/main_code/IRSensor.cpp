#include "IRSensor.h"

IRSensor::IRSensor(int analog_pin)
{
  setAnalogPin(analog_pin);
}

void IRSensor::setAnalogPin(int pin)
{
  this->analog_pin = pin;
}

void IRSensor::begin()
{
  
}

float IRSensor::read()
{
  int data = analogRead(analog_pin);
  return (data / 1023.0);
}

int IRSensor::readRaw()
{
  return analogRead(analog_pin);
}
