#include "UNO2ESPComm.h"

UNO2ESPComm esp;

UNO2ESPComm::UNO2ESPComm(uint8_t addr)
{
  setAddr(addr);
}

void UNO2ESPComm::begin(uint8_t addr)
{
  if(addr < 0)
  {
    Serial.println("invalid I2C address for UNO!");
    while(1);
  }
  this->addr = addr;
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
//  Serial.print("\tgripper_success:\t"); Serial.println((TX_data.gripper_success ? "true" : "false"));
}

void UNO2ESPComm::printTXData()
{
  Serial.println();
  Serial.println("Recieved from ESP:\n");
  Serial.print("\tgripper_signal: "); Serial.println(RX_data.gripper_signal);
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

const int min_pwm = 70; 
int UNO2ESPComm::getMotorInput(Direction side)
{
  if(rx_time_out_flag) // return zero if esp stops communicating
    return 0;

  // The final value is combination of forward and angular speed e.g. when
  // turning CCW the left wheel turns a bit slower than movement speed and
  // the right wheel turns slightly faster.
  float value = RX_data.motor_control_linear;
  if(side == LEFT)
    value -= (0.5 * RX_data.motor_control_angular);
  else // side == RIGHT
    value += (0.5 * RX_data.motor_control_angular);

  // Clamp value between -1.0 and 1.0
  if(value > 1)
    value = 1;
  else if(value < -1)
    value = -1;

  // map from (-1, 1) to (-255, 255)
  return value * 255;
}

void UNO2ESPComm::setFieldValues(bool* vals)
{
  setFieldValues(vals[0], vals[1], vals[2], vals[3]);
}

bool UNO2ESPComm::stopOnEdge()
{
  return RX_data.stop_on_edge;
}

bool UNO2ESPComm::data_received = false;
bool UNO2ESPComm::data_requested = false;
UNO2ESPComm::ESP_RX UNO2ESPComm::RX_data;
UNO2ESPComm::ESP_TX UNO2ESPComm::TX_data;
