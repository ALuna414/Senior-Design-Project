#include "UNO2ESPComm.h"

void setup() {
  Serial.begin(115200);
  esp.begin(0x55);
  Serial.println("\nWaiting for msg from ESP32...");
}

void loop() {
  while(!esp.scan())
  {
    if(Serial.available())
    {
      switch(Serial.read())
      {
      case 'f':
      {
        int i = Serial.parseInt();
        esp.TX_data.field_detection ^= (1<<i);
        break;
      }
      case 'g':
      {
        esp.TX_data.gripper_success = !esp.TX_data.gripper_success;
        break;
      }
      case 'p':
      {
        esp.TX_data.power_low = !esp.TX_data.power_low;
        break;
      }
      default:
        while(Serial.available()) Serial.read();
        break;
      }
      esp.printRXData();
      break;
    }
  }
  Serial.println("Message received!");
  uint8_t type = esp.eventType();
  if(type & 1)
  {
    esp.printTXData();
  }
  if(type & 2)
  {
    esp.printRXData();
  }
}
