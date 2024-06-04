#include <Servo.h>
#include "UNO2ESPComm.h"

#define SERVO1 A2

//start UI
#define CAMERA_MODEL_AI_THINKER
const int LED_START_INDICATOR = 7;
const int SWITCH_0 = 2;
const int SWITCH_1 = 17; //A3
const int BUTTON_START = 4;

//IR 
const int pinIRa = A1;
const int pinIRb = A0;
const int SIZE = 5;
int IRa_last_readings[SIZE];
int IRb_last_readings[SIZE];
int IRaIndex = 0;
int IRbIndex = 0;
bool IR_debug = true;

//motors
const int in1 = 5;
const int in2 = 11;
const int in3 = 6;
const int in4 = 3;

bool debug = false;
int IRvalueA = 0;
int IRvalueB = 0;
int initial_readingsA[5];
int initial_readingsB[5];

bool dataOut[4] = {true, false, true, false}; // Initial state is red for both sensors

int maxInitialValueA = 0;
int maxInitialValueB = 0;
float minThresholdA = 0;  // Declare as global variables
float maxThresholdA = 0;
float minThresholdB = 0;
float maxThresholdB = 0;
int left = 0;
int right = 0;
//part of the setup is how upon start, the bot will wait 5 seconds and then start moving. 
//During these 5 seconds, the initial_readings array will be filled with values for the red square

Servo myservo;

enum GripperState
{
  opened,
  closed
};

GripperState gripper_state;

void handleMotors();
void handleIR();
void handleGripper();
void doOnBlack();
int motorsMapDeadzone(int pwm);
void setMotors(int left, int right);
void serialMotorControl();

void setup()
{
  Serial.begin(115200);

  pinMode(pinIRa, INPUT);
  pinMode(pinIRb, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);


  //start UI
  pinMode(LED_START_INDICATOR, OUTPUT);
  pinMode(SWITCH_0, INPUT_PULLUP);
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(BUTTON_START, INPUT_PULLUP);
  digitalWrite(LED_START_INDICATOR, LOW);

  myservo.attach(SERVO1);

  for (int i = 0; i < SIZE; i++) {
    IRa_last_readings[i] = 0;
    IRb_last_readings[i] = 0;
  }

  for (int i = 0; i < 5; i++) {
    initial_readingsA[i] = analogRead(pinIRa);
    initial_readingsB[i] = analogRead(pinIRb);

    if (initial_readingsA[i] > maxInitialValueA) {
      maxInitialValueA = initial_readingsA[i]; // Find the maximum value from initial readings for sensor A
    }

    if (initial_readingsB[i] > maxInitialValueB) {
      maxInitialValueB = initial_readingsB[i]; // Find the maximum value from initial readings for sensor B
    }

    delay(500); // Delay between readings
  }

  // Calculate thresholds based on the maximum initial value
  minThresholdA = maxInitialValueA * 1.1;
  maxThresholdA = maxInitialValueA * 2;  
  
  minThresholdB = maxInitialValueB * 1.1;
  maxThresholdB = maxInitialValueB * 2;
// 


  Serial.println("Thresholds for Sensor A:");
  Serial.print("Min: ");
  Serial.println(minThresholdA);
  Serial.print("Max: ");
  Serial.println(maxThresholdA);

  Serial.println("Thresholds for Sensor B:");
  Serial.print("Min: ");
  Serial.println(minThresholdB);
  Serial.print("Max: ");
  Serial.println(maxThresholdB);

  myservo.write(100);

  esp.begin(0x55);
  
  while(digitalRead(BUTTON_START))
  {
    esp.scan();// wait for user to set switches until they press the button
    int color_target = ReadColorButtons();
    esp.TX_data.egg_color = color_target;
  }
  StartSequence();
  
}

void loop()
{
//   serialMotorControl();


//  IRvalueA = analogRead(pinIRa);
//  IRvalueB = analogRead(pinIRb);
//  
//  Serial.print("right sensor: ");
//  Serial.print(IRvalueA); Serial.print(", ");
//  if (isGray(IRvalueA, minThresholdA, maxThresholdA)) {
//    Serial.print("gray");
//  } else if (isBlack(IRvalueA, maxThresholdA)) {
//    Serial.print("black");
//  } else {
//    Serial.print("red");
//  }
//
//  Serial.print("\t left sensor: ");
//  Serial.print(IRvalueB); Serial.print(", ");
//  if (isGray(IRvalueB, minThresholdB, maxThresholdB)) {
//    Serial.println("gray");
//  } else if (isBlack(IRvalueB, maxThresholdB)) {
//    Serial.println("black");
//  } else {
//    Serial.println("red");
//  }
//
//  delay(250);
  
  esp.scan(); 
  
  handleMotors();
  handleIR();
  handleGripper();
}

void BlinkStartLED() {
  Serial.println("Blinking START LED!");
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_START_INDICATOR, HIGH);
    delay(500);
    digitalWrite(LED_START_INDICATOR, LOW);
    delay(500);
  }
  //digitalWrite(LED_START_INDICATOR, HIGH);
}

void StartSequence() {
  
  Serial.print("color-number is: "); Serial.println(esp.TX_data.egg_color);
  //  BlinkStartLED();
  esp.TX_data.start = true;
}

int ReadColorButtons() {
  int bit0 = digitalRead(SWITCH_0); // bits are 0 or 1
  int bit1 = digitalRead(SWITCH_1);

  return (bit1 << 1) | bit0;  //returns int 0, 1, 2, or 3
}

void handleMotors()
{
  int left = esp.getMotorInput(LEFT);
  int right = esp.getMotorInput(RIGHT);
  Serial.print("left, right: "); Serial.print(left); Serial.print('\t'); Serial.println(right);
  setMotors(left, right);
  
  if(esp.rx_time_out_flag)
    Serial.println("RX Time OUT!");
}
void handleIR()
{
  IRvalueA = analogRead(pinIRa);
  IRvalueB = analogRead(pinIRb);
  
  IRa_last_readings[IRaIndex] = IRvalueA;
  IRb_last_readings[IRbIndex] = IRvalueB;

  IRaIndex = (IRaIndex + 1) % SIZE;
  IRbIndex = (IRbIndex + 1) % SIZE;

  float avg_A = 0;
  float avg_B = 0;

  for (int i = 0; i < SIZE; i++){
    avg_A += IRa_last_readings[i];
    avg_B += IRb_last_readings[i];
  }

  avg_A /= SIZE;
  avg_B /= SIZE;
  if (IR_debug){
    Serial.print("left sensor: ");
    Serial.print(IRvalueA); Serial.print("\n");
    Serial.print("left sensor AVERAGE: ");
    Serial.print(avg_A); Serial.print("\nleft sensor sees: ");
  }


  // Check and update the color status for sensor A
  if (isGray(avg_A, minThresholdA, maxThresholdA)) {
    dataOut[0] = false;
    dataOut[1] = false;
    Serial.println("gray\n");

  } else if (isBlack(avg_A, maxThresholdA)) {
    dataOut[0] = false;
    dataOut[1] = true; 
    Serial.println("black\n");
    doOnBlack();
  } else {
    dataOut[0] = true;
    dataOut[1] = false;
    Serial.println("red\n");

  }
  if(IR_debug){
    Serial.print("right sensor: ");
    Serial.print(IRvalueB); Serial.print("\n");
    Serial.print("right sensor AVERAGE: ");
    Serial.print(avg_B); Serial.print("\nright sensor sees: ");
  }


  // Check and update the color status for sensor B
  if (isGray(avg_B, minThresholdB, maxThresholdB)) {
    dataOut[2] = false;
    dataOut[3] = false;
    Serial.println("gray\n");
  } else if (isBlack(avg_B, maxThresholdB)) {
    dataOut[2] = false;
    dataOut[3] = true;
    doOnBlack();
    Serial.println("black\n");
  } else {
    dataOut[2] = true;
    dataOut[3] = false;
    Serial.println("red\n");
  }

  esp.setFieldValues(dataOut);
}
void handleGripper()
{
  const int pos_opened = 100;
  const int pos_closed = 75;
  
  if(esp.RX_data.gripper_signal && gripper_state == opened)
  {
    esp.TX_data.gripper_active = true;
    for (int pos = pos_opened; pos >= pos_closed; pos -= 2)  // goes from 0 degrees to 180 degrees    // in steps of 1 degree
    {
       myservo.write(pos);              // tell servo to go to position in variable 'pos'
       delay(50);                       // waits 50ms for the servo to reach the position
    }
    gripper_state = closed;
    esp.TX_data.gripper_active = false;
  }
  else if(!esp.RX_data.gripper_signal && gripper_state == closed)
  {
    esp.TX_data.gripper_active = true;
    for (int pos = pos_closed; pos <= pos_opened; pos += 2)  // goes from close degrees to open degrees    // in steps of 1 degree
    {
       myservo.write(pos);              // tell servo to go to position in variable 'pos'
       delay(50);                       // waits 50ms for the servo to reach the position
    } 
    gripper_state = opened;
    esp.TX_data.gripper_active = false;
  }
}

int motorsMapDeadzone(int pwm)
{
  const int minPWM = 50;
  const int maxPWM = 255;
  
  if(pwm == 0)
    return 0;  
  
  bool neg = pwm < 0;
  if(neg)
    pwm *= -1;

  pwm = minPWM + pwm * ((maxPWM - minPWM) / (float)maxPWM);

  if(neg)
    pwm *= -1;

  return pwm;
}

void serialMotorControl()
{
  switch(Serial.read())
  {
    case 'l':
    {
      left = Serial.parseInt();
      break;
    }
    case 'r':
    {
      right = Serial.parseInt();
      break;
    }
    case 'b':
    {
      right = left = Serial.parseInt();
      break;
    }
    // default:
    //   left = right = 0;
  }
  setMotors(left, right);
}

void setMotors(int left, int right)
{
  left = motorsMapDeadzone(left);
  right = motorsMapDeadzone(right);
  Serial.print("mapped     : "); Serial.print(left); Serial.print('\t'); Serial.println(right);
  
  if(esp.rx_time_out_flag)
  {
    left = 0;
    right = 0;
    Serial.println("timeout in setMotors!");
  }
  if(left < 0)
  {
    digitalWrite(in4, LOW);
    analogWrite(in3, -left);
//    Serial.println("left backwards");
  }
  else
  {
    digitalWrite(in3, LOW);
    analogWrite(in4, left);
//    Serial.println("left forwards");
  }

  if(right < 0)
  {
    digitalWrite(in2, LOW);
    analogWrite(in1, -right);
//    Serial.println("right backwards");
  }
  else
  {
    digitalWrite(in1, LOW);
    analogWrite(in2, right);
//    Serial.println("right forwards");
  }
}

bool isGray(int value, float minThreshold, float maxThreshold) {
  return (value >= minThreshold && value <= maxThreshold);
}

bool isBlack(int value, float threshold) {
  return (value > threshold);
}

void doOnBlack()
{
  if(!esp.rx_time_out_flag && esp.stopOnEdge())
  {
    setMotors(-180, -180);
    delay(100);
    setMotors(0, 0);
    
    esp.rx_time_out_flag = true;
  }
}
