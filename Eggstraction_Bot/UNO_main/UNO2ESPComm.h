#ifndef UNO_2_ESP_COMM
#define UNO_2_ESP_COMM

#include <arduino.h>
#include <Wire.h>


/*
//Example minimum implementation:

void setup()
{
  Serial.begin(155200);
  esp.begin(0x55);
}

void loop()
{
  esp.scan(); // should happen often enough to not trigger receiver timeout
}
 */

enum Direction
{
  LEFT,
  RIGHT
};

class UNO2ESPComm
{
private:
  uint8_t addr = -1; // address must be set before the begin function is called,
                     // either in the object constructor, setAddr(), or as an arg
                     // to begin itself

  // used internally, reset with scan function
  static bool data_received;
  static bool data_requested;

  // these stay set until client checks with the eventType function
  bool has_received = false;
  bool has_requested = false;

  long last_rx_msg_time = 0;
  int rx_time_out_value = 1200;
  
  
public:
  UNO2ESPComm(){}
  UNO2ESPComm(uint8_t addr);

  void begin(uint8_t addr = -1);

  bool rx_time_out_flag;
  
  bool scan();  // must be called frequently to check for data in the receive buffer
  uint8_t eventType();  // can be called whenever, returns 0 for no event, 1 for receive, 2 for request, or 3 for both
  void setAddr(uint8_t addr);
  void printRXData();
  void printTXData();
  void setRXTimeOut(int ms);

  // static functions passed to the Wire library, called in ISR routines
  static void wireReceiveHandler(int num_bytes);
  static void wireRequestHandler();

  // helper functions
  void setFieldValues(bool left_bit1, bool left_bit0, bool right_bit1, bool right_bit0);
  void setFieldValues(bool* vals);
  int getMotorInput(Direction side);
  bool stopOnEdge();

  // the mirror of the esp's UNO_TX struct
  struct __attribute__((packed)) ESP_RX
  {
    bool gripper_signal;
    float motor_control_linear;
    float motor_control_angular;
    bool stop_on_edge;
  } static RX_data; // the actual variable is called RX_data
  
  // the mirror of the esp's UNO_RX struct
  struct __attribute__((packed)) ESP_TX
  {
    uint8_t field_detection;
    bool gripper_active = false;
    bool start = false;
    int8_t egg_color = -1; //0: pink, 1: yellow, 2: green, 3: blue
  } static TX_data;
};

extern UNO2ESPComm esp; // global esp object

//get motor control data:
// linear float: esp.RX_data.motor_control_linear
// angular float: esp.RX_data.motor_contorl_angular

#endif
