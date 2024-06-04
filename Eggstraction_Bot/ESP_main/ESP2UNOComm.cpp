#include "ESP2UNOComm.h"

ESP2UNOComm uno;

ESP2UNOComm::ESP2UNOComm(uint8_t uno_addr)
{
  setAddr(uno_addr);
}

void ESP2UNOComm::begin(uint8_t uno_addr)
{
  if(uno_addr < 0)
  {
    Serial.println("invalid I2C address for UNO!");
    while(1);
  }
  this->uno_addr = uno_addr;
  Wire.begin(14, 15, 0); //Initializing I2C Pin Numbers on ESP32-CAM Data, Clock - Blue Green

  Serial.print("Searching for UNO at I2C address 0x");
  if(uno_addr < 16) Serial.print('0');
  Serial.print(uno_addr, HEX); Serial.print("... ");
  uint8_t error_code = 1;
  int counter = 0;
  while(error_code)
  {
    Wire.beginTransmission(uno_addr);
    error_code = Wire.endTransmission();
    Serial.print('|');
    delay(250);
    if(counter++ > 20)
    {
      Serial.println();
      Serial.print("UNO not found!");
      while(1);
    }
  }
  Serial.println(" Success!");
}

void ESP2UNOComm::sendData()
{
  Wire.beginTransmission(uno_addr);
  Wire.write((uint8_t*)&TX_data, sizeof(UNO_TX));
  Wire.endTransmission();
}

void ESP2UNOComm::requestData()
{
  Wire.requestFrom(uno_addr, sizeof(UNO_RX));
  while(!Wire.available()){}
  Wire.readBytes((uint8_t*)&RX_data, sizeof(UNO_RX));
}

void ESP2UNOComm::setAddr(uint8_t uno_addr)
{
  this->uno_addr = uno_addr;
}

void ESP2UNOComm::setMotorsLinear(float val)
{
  TX_data.motor_control_linear = val;
}

void ESP2UNOComm::setMotorsAngular(float val)
{
  TX_data.motor_control_angular = val;
}

void ESP2UNOComm::setMotors(float linear, float angular)
{
  setMotorsLinear(linear);
  setMotorsAngular(angular);
}

IRColor ESP2UNOComm::getLeftIRSensor()
{
  uint8_t val = (RX_data.field_detection & B1100) >> 2;
  
  if(val == 0)
    return IRColor::GRAY;
  if(val == 1)
    return IRColor::BLACK;
  if(val == 2)
    return IRColor::RED;
    
  return IRColor::ERR;
}

IRColor ESP2UNOComm::getRightIRSensor()
{
  uint8_t val = (RX_data.field_detection & B0011);
  
  if(val == 0)
    return IRColor::GRAY;
  if(val == 1)
    return IRColor::BLACK;
  if(val == 2)
    return IRColor::RED;
    
  return IRColor::ERR;
}

void ESP2UNOComm::printRXData()
{
  Serial.println();
  Serial.println("Data from UNO: \n");
  Serial.println("field_detection:\t"); Serial.println(RX_data.field_detection, BIN);
//  Serial.print("gripper_success:\t"); Serial.println((RX_data.gripper_success ? "true" : "false"));
}

void ESP2UNOComm::printTXData()
{
  Serial.println();
  Serial.println("Data sent to UNO: \n");
  Serial.println("gripper_signal:\t"); Serial.println((TX_data.gripper_signal ? "true" : "false"));
  Serial.print("motor_control_linear:\t"); Serial.println(TX_data.motor_control_linear);
  Serial.print("motor_control_angular:\t"); Serial.println(TX_data.motor_control_angular);
}
