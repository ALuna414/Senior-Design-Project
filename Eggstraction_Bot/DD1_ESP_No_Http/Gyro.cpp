#include "Gyro.h"

Gyro::Gyro(uint8_t addr)
{
  this->addr = addr;
}

bool Gyro::begin(int scale_num)
{
  // Check for MPU6050
  Wire.beginTransmission(addr);
  if(Wire.endTransmission())
    return false;
    
  // reset MPU 6050
  Wire.beginTransmission(addr);
  Wire.write(pwr_mgmt_reg);
  Wire.write(0);
  Wire.endTransmission();

  // set gyro scale

  max_scale = scale_num;
  Wire.beginTransmission(addr);
  Wire.write(config_reg);
  Wire.write(0 | (max_scale << 3));
  Wire.endTransmission();

  timer = micros();
  delayMicroseconds(1000);
  calibrate(500);
  return true;
}

int Gyro::getRawData()
{
  // set the module's internal reg pointer to the gyro Z axis register
  Wire.beginTransmission(addr);
  Wire.write(data_reg);
  Wire.endTransmission();
  
  // request the 2 8-bit data bytes
  Wire.requestFrom(addr, 2);

  // wait until the data has been recieved or 100ms has elapsed
  int t = millis();
  const int time_out = 100;
  while(!Wire.available() && (millis() - t < time_out)){}

  // logical OR the most significant and least significant bytes, shifting the MS byte 8 bits to the left.
  // The MS byte is always sent first, and the read functions are always called in the same order, so the
  // left Wire.read() will always be the MS byte and the right Wire.read() the LS byte.
  raw_MS = Wire.read();
  raw_LS = Wire.read();
  raw_data = (raw_MS << 8) | (raw_LS);
  return raw_data;
}

void Gyro::update()
{
  // dt is the elapsed time in seconds since the last time
  // update (this function) was called.
  long time_stamp = micros();
  float dt = (time_stamp - timer) / 1000000.0;
  timer = time_stamp;

  // magic number 32758.0 converts the raw data from an integer
  // [-32768, 32767] to a float [-1.0, 1.0]. Multiplying by the
  // appropriate scale_factor converts rate to [-max_scale, max_scale]
  // in degrees per second.
  rate = getRawData() / 32768.0;
  rate *= scale_factor[max_scale];
  rate += rate_offset;

  // the integration step: current angle is incremented by the current
  // rate times the elapsed time. This results in a small error when the
  // rate is changing, but this error is cancelled out whenever the rate
  // returns to zero.
  angle += rate * dt;

  // angle is contstrained to [0, 360) degrees
  angle = wrap360(angle);
}

void Gyro::calibrate(int iterations)
{
  Serial.println("Calibrating...");

  // sum the raw rate data at 400Hz. An int is sufficiently large because
  // the error will generally be much smaller than the maximum value.
  int rate_accumulator = 0;
  for(int i = 0; i < iterations; i++)
  {
    rate_accumulator += getRawData();
    delayMicroseconds(2500);
  }

  // calculate the average rate error of the gyro. The negative of this 
  // result will be added to every value so that the gyro reads 0 when
  // motionless.
  float avg_rate_error = rate_accumulator / (float)iterations;
  rate_offset = -avg_rate_error / 32.768;

  Serial.print("Complete! calculated offset:");
  Serial.print(rate_offset);
}

float Gyro::wrap360(float num)
{
  // Possible but unlikely long loop times here, but I'm pretty sure an
  // infinite loop is impossible.
  while(num > 360)
    num -= 360;
  while(num < 0)
    num += 360;

  return num;
}
