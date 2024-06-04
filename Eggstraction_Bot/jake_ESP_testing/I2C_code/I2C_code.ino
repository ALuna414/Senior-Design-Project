#include <Wire.h>
#include "Gyro.h"

Gyro gyro;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if(!gyro.begin(3))
  {
    Serial.println("Gyro not found!");
    while(1);
  }
  gyro.calibrate(50);
  
}

void loop() {
  for(int i = 0; i < 25; i++)
  {
    gyro.update();
    delayMicroseconds(2500);
  }

//  Serial.print(gyro.raw_MS);
//  Serial.print('\t');
//  Serial.print(gyro.raw_LS);
//  Serial.print('\t');
//  Serial.print(gyro.raw_data);
//  Serial.print('\t');
//  Serial.println(gyro.rate);

  Serial.print(gyro.rate);
  Serial.print('\t');
  Serial.println(gyro.angle);
}
