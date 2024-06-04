#ifndef ESP_2_UNO_COMM
#define ESP_2_UNO_COMM

#include <arduino.h>
#include <Wire.h>

class ESP2UNOComm
{
private:
  uint8_t uno_addr = -1; // uno address must be set before the begin function is called,
                     // either in the object constructor, setAddr(), or as an arg
                     // to begin itself

public:

  enum class IRColor
  {
    GRAY,
    BLACK,
    RED,
    ERR
  };
  
  ESP2UNOComm(){}
  ESP2UNOComm(uint8_t uno_addr);

  void begin(uint8_t uno_addr = -1);

  void sendData();  // transmits current TX_data to the UNO
  void requestData(); // sends a request for an RX_data update, then updates RX_data
  void setAddr(uint8_t uno_addr);
  void printTXData();
  void printRXData();

  // helper functions
  void setMotorsLinear(float val);
  void setMotorsAngular(float val);
  void setMotors(float linear, float angular);
  IRColor getLeftIRSensor();
  IRColor getRightIRSensor();

  // the mirror of the UNO's ESP_RX struct
  struct __attribute__((packed)) UNO_TX
  {
    bool grabber_signal;
    float motor_control_linear;
    float motor_control_angular;
    bool stop_on_edge;
  } TX_data; // the actual variable is called TX_data

  // the mirror of the UNO's ESP_TX struct
  struct __attribute__((packed)) UNO_RX
  {
    uint8_t field_detection;
    bool gripper_success = false;
    bool power_low = true;
  } RX_data;
};

extern ESP2UNOComm uno; // global uno object

#endif
