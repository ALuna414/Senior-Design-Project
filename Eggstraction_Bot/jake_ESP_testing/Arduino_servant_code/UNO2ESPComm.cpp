#include "UNO2ESPComm.h"

UNO2ESPComm esp;

UNO2ESPComm::UNO2ESPComm(uint8_t addr)
{
  setAddr(addr);
}

void UNO2ESPComm::begin(uint8_t addr)
{
  if(this->addr >= 0)
    addr = this->addr;
  if(addr < 0)
  {
    Serial.println("invalid I2C address for UNO!");
    while(1);
  }
  Wire.begin(addr);
  Wire.onReceive(wireReceiveHandler);
  Wire.onRequest(wireRequestHandler);
  Serial.print("UNO is now an I2C slave device with address 0x");
  if(addr < 16) Serial.print('0');
  Serial.println(addr, HEX);
}

bool UNO2ESPComm::scan()
{
  bool esp_interaction = false;
  if(data_received)
  {
    Wire.readBytes((uint8_t*)&RX_data, sizeof(ESP_RX));
    data_received = false;
    has_received = true;
    esp_interaction = true;
    last_rx_msg_time = millis();
    rx_time_out_flag = false;
  }
  else
  {
    if(millis() - last_rx_msg_time > rx_time_out_value)
      rx_time_out_flag = true;
  }
  if(data_requested)
  {
    data_requested = false;
    has_requested = true;
    esp_interaction = true;
  }
  return esp_interaction;
}

uint8_t UNO2ESPComm::eventType()
{
  int type = 0;
  if(has_received)
    type += 1;
  if(has_requested)
    type += 2;

  has_requested = has_received = false;
  return type;
}

void UNO2ESPComm::setAddr(uint8_t addr)
{
  this->addr = addr;
}

void UNO2ESPComm::setRXTimeOut(int ms)
{
  rx_time_out_value = ms;
}

void UNO2ESPComm::printRXData()
{
  Serial.println();
  Serial.println("Sent to ESP:\n");
  Serial.println("\tfield_detection:\t"); Serial.println(TX_data.field_detection, BIN);
  Serial.print("\tgripper_success:\t"); Serial.println((TX_data.gripper_success ? "true" : "false"));
  Serial.print("\tpower_low:\t"); Serial.println((TX_data.power_low ? "true" : "false"));
}

void UNO2ESPComm::printTXData()
{
  Serial.println();
  Serial.println("Recieved from ESP:\n");
  Serial.print("\tgrabber_signal: "); Serial.println(RX_data.grabber_signal);
  Serial.print("\tmotor_control_linear: "); Serial.println(RX_data.motor_control_linear);
  Serial.print("\tmotor_control_angular: "); Serial.println(RX_data.motor_control_angular);
}

void UNO2ESPComm::wireReceiveHandler(int num_bytes)
{                                                                 
  data_received = true;
}

void UNO2ESPComm::wireRequestHandler()
{
  data_requested = true;
  Wire.write((uint8_t*)&TX_data, sizeof(ESP_TX));
}

void UNO2ESPComm::setFieldValues(bool left_bit1, bool left_bit0, bool right_bit1, bool right_bit0)
{
  TX_data.field_detection = 0;
  
  if(left_bit1)
    TX_data.field_detection |= (1 << 3);

  if(left_bit0)
    TX_data.field_detection |= (1 << 2);

  if(right_bit1)
    TX_data.field_detection |= (1 << 1);

  if(right_bit0)
    TX_data.field_detection |= 1;
}
int UNO2ESPComm::getLeftMotorInput()
{
  if(rx_time_out_value) // return zero if esp stops communicating
    return 0;
  
  int value = RX_data.motor_control_linear - 0.5 * RX_data.motor_control_angular;
  if(value > 1)
    value = 1;
  else if(value < -1)
    value = -1;
  return value * 255;
}
int UNO2ESPComm::getRightMotorInput()
{
  if(rx_time_out_value) // return zero if esp stops communicating
    return 0;
  
  int value = RX_data.motor_control_linear + 0.5 * RX_data.motor_control_angular;
  if(value > 1)
    value = 1;
  else if(value < -1)
    value = -1;
  return value * 255;
}

bool UNO2ESPComm::data_received = false;
bool UNO2ESPComm::data_requested = false;
UNO2ESPComm::ESP_RX UNO2ESPComm::RX_data;
UNO2ESPComm::ESP_TX UNO2ESPComm::TX_data;
