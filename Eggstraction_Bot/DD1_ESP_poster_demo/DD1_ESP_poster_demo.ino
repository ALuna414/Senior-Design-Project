 #include "Gyro.h"
#include "ESP2UNOComm.h"
#include "RotationPID.h"

Gyro gyro;
RotationPID rotPID(0.02, 0, 0);

void navUpdate();
void updatePID(float dt);

void setup() {
  Serial.begin(115200);
  Serial.println("Start of setup");
  uno.begin(0x55);
  gyro.begin();

  gyro.calibrate(100);
}

void loop() {
  for(int i = 0; i < 10; i++)
  {
    gyro.update();
    delayMicroseconds(2500);
  }

  uno.requestData();
  navUpdate();
  uno.sendData();
}

enum State
{
  turning,
  straight
};

State driving_state;


ESP2UNOComm::IRColor left_IR;
ESP2UNOComm::IRColor right_IR;

void printIRData()
{
  Serial.print("IR data: left: ");
  switch(left_IR)
  {
    case ESP2UNOComm::IRColor::GRAY: Serial.print("Gray"); break;
    case ESP2UNOComm::IRColor::BLACK: Serial.print("black"); break;
    case ESP2UNOComm::IRColor::RED: Serial.print("red"); break;
  }
  Serial.print("\tright: ");
  switch(right_IR)
  {
    case ESP2UNOComm::IRColor::GRAY: Serial.print("Gray"); break;
    case ESP2UNOComm::IRColor::BLACK: Serial.print("black"); break;
    case ESP2UNOComm::IRColor::RED: Serial.print("red"); break;
  }
  Serial.println();
}


float target_speed = 0;
float linear_accel = 2;
void updateMotors(float dt)
{
  float speed = uno.TX_data.motor_control_linear;
  if(target_speed - speed < linear_accel * dt)
  {
    uno.setMotorsLinear(target_speed);
  }
  else
  {
    uno.setMotorsLinear(speed + linear_accel * dt);
  }
}
void navUpdate()
{
  static long prev_micros = 0;
  float dt = (micros() - prev_micros) / 1000000.0;
  prev_micros = micros();
  
  left_IR = uno.getLeftIRSensor();
  right_IR = uno.getRightIRSensor();

  //printIRData();

//  switch(driving_state)
//  {
//    case turning:
//    {
//      if(rotPID.getErrorABS() < 2)
//      {
//        target_speed = 0.75;
//        driving_state = straight;
//        uno.TX_data.stop_on_edge = true;
//      }
//      break;
//    }
//    case straight:
//    {
//      if(left_IR == ESP2UNOComm::IRColor::BLACK)
//      {
////        uno.setMotorsLinear(0);
//        target_speed = 0;
//        rotPID.setTargetAngle(gyro.wrap360(gyro.angle - 120));
//        driving_state = turning;
//        uno.TX_data.stop_on_edge = false;
//      }
//      else if(right_IR == ESP2UNOComm::IRColor::BLACK)
//      {
////        uno.setMotorsLinear(0);
//        target_speed = 0;
//        rotPID.setTargetAngle(gyro.wrap360(gyro.angle + 120));
//        driving_state = turning;
//        uno.TX_data.stop_on_edge = false;
//      }
//      break;
//    }
//  }

  Serial.print("heading: "); Serial.print(gyro.angle); 
//  Serial.print("\ttarget: "); Serial.print(rotPID.target_angle);
//  Serial.print("\tvirt target: "); Serial.print(rotPID.virtual_target);Serial.print("\terror: "); Serial.println(rotPID.getErrorABS());

  updateMotors(dt);
  uno.setMotorsAngular(rotPID.update(dt, gyro.angle));
}
