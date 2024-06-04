#include "Gyro.h"

Gyro::Gyro(uint8_t addr)
{
  this->addr = addr;
}

void Gyro::begin(int scale_num)
{
  // reset MPU 6050
  Wire.beginTransmission(addr);
  Wire.write(pwr_mgmt_reg);
  Wire.write(0);
  Wire.endTransmission();

  // set gyro scale +- 1000 deg/s
  Wire.beginTransmission(addr);
  Wire.write(config_reg);
  Wire.write(0 | (scale_num << 3));
  Wire.endTransmission();

  timer = micros();
  delayMicroseconds(1000);
  calibrate(500);
}

int Gyro::getRawData()
{
  Wire.beginTransmission(addr);
  Wire.write(data_reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 2);

  int t = millis();
  const int time_out = 100;
  while(!Wire.available() && (millis() - t < time_out)){}
  
  return (Wire.read() << 8) | (Wire.read());
}

void Gyro::update()
{
  long time_stamp = micros();
  float dt = (time_stamp - timer) / 1000000.0;
  timer = time_stamp;
  
  rate = getRawData() / 32.768;
  rate += rate_offset;

  angle += rate * dt;
  angle = wrap360(angle);
}

void Gyro::calibrate(int iterations)
{
  Serial.println("Calibrating...");
  int rate_accumulator = 0;
  for(int i = 0; i < iterations; i++)
  {
    rate_accumulator += getRawData();
    delayMicroseconds(2500);
  }

  float avg_rate_error = rate_accumulator / (float)iterations;
  rate_offset = -avg_rate_error / 32.768;

  Serial.print("Complete! calculated offset:");
  Serial.print(rate_offset);
}

float Gyro::wrap360(float num)
{
  while(num > 360)
    num -= 360;
  while(num < 0)
    num += 360;

  return num;
}
