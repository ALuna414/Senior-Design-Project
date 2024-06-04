#ifndef GYRO_
#define GYRO_

#include <Wire.h>
#include <Arduino.h>

class Gyro
{
private:
  int getRawData(); // get the Z axis gyro data from the gyroscope
  float rate_offset = 0;  // calculated to calibrate gyro output to 0 when motionless
  long timer = 0; // records prev time to calculate elapsed time between update calls
  
  const uint8_t config_reg = 0x1B;    //
  const uint8_t data_reg = 0x47;      // MPU6050 internal register addresses
  const uint8_t pwr_mgmt_reg = 0x6B;  //

  int scale_factor[4] = {250, 500, 1000, 2000}; // max scale factors in deg/s
  int max_scale = 0;
  
public:
  uint8_t addr = 0x68;  // gyro module I2C address
  
  float rate = 0;   // curent rate in degrees/s
  float angle = 0;  // current angle in degrees

  int16_t raw_data = 0;
  uint8_t raw_MS = 0;
  uint8_t raw_LS = 0;
  
  Gyro(){}
  Gyro(uint8_t addr);

  bool begin(int scale_num = 2);  // initialize pins, set max scale:
                                  // 0: 250 deg/s
                                  // 1: 500 deg/s
                                  // 2: 1000 deg/s
                                  // 3: 2000 deg/s

  void calibrate(int iterations); // runs in begin() to estimate the gyroscope's drift error: can be run again to re-calibrate
  void update(); // run every loop to get new data and integrate old data

  float wrap360(float num); // constrain an angular value to [0, 360) degrees
  
};

#endif
