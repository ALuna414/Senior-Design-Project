#include "ESP2UNOComm.h"
#include "Gyro.h"

Gyro gyro(0x68);

void setup() {
  Serial.begin(115200);
  uno.begin(0x55);
  if( !gyro.begin())
  {
    Serial.println("gyro not found");
    while(1);
  }
}

void loop() {
  int counter = 0;
  while(!Serial.available())
  {
    gyro.update();
    delayMicroseconds(2500);
    if(++counter >= 100)
    {
      Serial.print("angle: "); Serial.print(gyro.angle);
      Serial.print("\trate: "); Serial.println(gyro.rate);
      counter = 0;
    }
  }
  switch(Serial.read())
  {
  case 'w': // write data
  {
    uno.TX_data.motor_control_linear = Serial.parseInt();
    uno.sendData();
    Serial.println("UNO updated");
    break;
  }
  case 'r': // request data
   {
    uno.requestData();
    uno.printRXData();
   }
  }
}
