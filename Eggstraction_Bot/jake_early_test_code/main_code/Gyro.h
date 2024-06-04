#ifndef GYRO_
#define GYRO_

#include <Wire.h>
#include <Arduino.h>

class Gyro
{
private:
  int getRawData(); // get the Z axis gyro data from the gyroscope
  
public:
  uint8_t addr = 0x68;
  const uint8_t config_reg = 0x1B;
  const uint8_t data_reg = 0x47;
  const uint8_t pwr_mgmt_reg = 0x6B;

  float rate_offset = 0;
  float rate = 0;
  float angle = 0;

  long timer = 0;
  
  Gyro(){}
  Gyro(uint8_t addr);

  void begin(int scale_num = 0); // initialize pins

  void calibrate(int iterations); // run to acquire the gyroscope's drift value
  void update(); // run everyloop to get new data and integrate old data

  float wrap360(float num); // wrap an angular value so it stays within 360 degrees
  
};

#endif
