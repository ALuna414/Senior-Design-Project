#include "Gyro.h"
#include "ESP2UNOComm.h"
#include "RotationPID.h"

// Obj Detection Library Includes
#define MAX_RESOLUTION_VGA
#define MAX_RECURSION_DEPTH 13

#include "esp32cam.h"
#include "esp32cam/JpegDecoder.h"
#include "esp32cam/apps/ColorBlobDetector.h"

using namespace Eloquent::Esp32cam;

Cam cam;
JpegDecoder decoder;
Applications::ColorBlobDetector detector(240, 140, 180);
 // 212 85 128 Pink Egg
// // End of Obj Detection Includes

Gyro gyro;
RotationPID rotPID(0.04, 0.00, 0, 0); // (P, I, D, deadzone deg)

TaskHandle_t Task1;

void navUpdate(float dt);
void updatePID(float dt);

float calculate_approx_angle(float center_of_blob)
{
  const float manual_correction_factor = 0.95;

  return ((center_of_blob - 40) * 33 / 40) * manual_correction_factor;
}

// (entire screen), it is default selection and an egg isn't actually being seen
float BlobArea(float left, float right, float top, float bottom)
{
  return((right - left) * (bottom - top));
}

void trackEgg(float egg_angle, float dt);
float egg_angle = 0;
bool egg_detected = false;

void Loop2(void * parameter)
{
//  delay(10000);
  while(1)
  {
    static long prev_micros = 0;
    float dt = (micros() - prev_micros) / 1000000.0;
    prev_micros = micros();
    
    for(int i = 0; i < 10; i++)
    {
      gyro.update();
      delayMicroseconds(2500);
    }
    
    uno.requestData();
    navUpdate(dt);
    uno.sendData();

//  void printBlobBorders();
  }
}

void printBlobBorders()
{
  Serial.print("blob data l,r,t,b: ");
  Serial.print(detector.blob.left); Serial.print(' ');
  Serial.print(detector.blob.right); Serial.print(' ');
  Serial.print(detector.blob.top); Serial.print(' ');
  Serial.println(detector.blob.bottom);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start of setup");
  
  cam.aithinker();
  cam.highQuality();
  cam.vga();
  cam.highestSaturation();
  cam.disableAutomaticWhiteBalance();
  cam.disableAutomaticExposureControl();
  cam.disableGainControl();
  detector.tollerate(45);
  detector.setMinArea(7 * 7);
  while (!cam.begin())
    Serial.println(cam.getErrorMessage());
  
  uno.begin(0x55);
  gyro.begin();
  gyro.calibrate(100);

  xTaskCreatePinnedToCore(
    Loop2,
    "Loop2",
    10000,
    NULL,
    0,
    &Task1,
    0
  );

  delay(500);
}

void loop()
{
  if (!cam.capture()) 
  {
    Serial.println(cam.getErrorMessage());
    return;
  }

  if (!decoder.decode(cam)) 
  {
    Serial.println(decoder.getErrorMessage());
    return;
  }
  
  float center_of_blob = (detector.blob.right + detector.blob.left)/2;
  float blob_area = BlobArea(detector.blob.left, detector.blob.right, detector.blob.top, detector.blob.bottom);
  Serial.print("BLOB AREA:");
  Serial.println(blob_area);
  
  egg_detected = true;
  // if(detector.blob.left == 0 && detector.blob.right == 0 && detector.blob.top == 0 && detector.blob.bottom == 0)
  // { 
  //   egg_detected = false;
  // }
  // if(detector.blob.left == 80 && detector.blob.right == 0 && detector.blob.top == 60 && detector.blob.bottom == 0)
  // {
  //   egg_detected = false;
  // }

  // while debugging, these may turn out useful
  Serial.print(detector.maskCount);
  Serial.println(" pixels match target color");
  Serial.println(detector.toString());
  detector.printTo(Serial);
  
  //if(egg_detected)
  egg_angle = calculate_approx_angle(center_of_blob);
    
  //else 
  //{
    //Serial.println("No BLOB Detected ... ... ...");
  //}
}

void updateMotors(float target_spd, float dt);

enum State
{
  scanning,
  searching,
  seeking,
  halt
};

State state = scanning;

void navUpdate(float dt)
{
  static float search_angle = 0;
  if(state != scanning) search_angle = gyro.angle;
  switch(state)
  {
    case scanning:
    {
      const float search_spd = 45;
      
      updateMotors(0, dt);
      
      search_angle += search_spd * dt;

      gyro.wrap360(search_angle);
      rotPID.setTargetAngle(search_angle);
      
      
      
      if(egg_detected)
      {
        uno.setMotorsAngular(0);
        updateMotors(0, dt);
        state = seeking;
      }
      
      break;
    }
    case seeking:
    {
      if(!egg_detected)
      {
        state = scanning;
        break;
      }
      float target_speed = 0;
      trackEgg(egg_angle, dt);
      if(detector.blob.bottom < 30)
        target_speed = 0.75;
      else if(detector.blob.bottom < 55)
        target_speed = 0.25;
      else
        state = halt;
      updateMotors(target_speed, dt);
      
      break;
    }
    case halt:
    {
      uno.TX_data.grabber_signal = true;
       
      updateMotors(0, dt);
      break;
    }
  }

  if(state != halt)
    uno.setMotorsAngular(rotPID.update(dt, gyro.angle));
}


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

float linear_accel = 2;
void updateMotors(float target_speed, float dt)
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

void trackEgg(float egg_angle, float dt)
{
  const float deadzone = 5;
  
  static float prev_egg_angle = 0;
  if(prev_egg_angle != egg_angle)
  {
    rotPID.setTargetAngle(gyro.wrap360(gyro.angle - egg_angle));
    prev_egg_angle = egg_angle;
  }

//  if(rotPID.getErrorABS() < deadzone)
//  {
//    uno.setMotorsAngular(0);
//    rotPID.accumulator = 0;
//  }
  else
    uno.setMotorsAngular(rotPID.update(dt, gyro.angle));
}

//void navUpdate()
//{
//  static long prev_micros = 0;
//  float dt = (micros() - prev_micros) / 1000000.0;
//  prev_micros = micros();
//  
//  left_IR = uno.getLeftIRSensor();
//  right_IR = uno.getRightIRSensor();
//
//  //printIRData();
//
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
//
//  Serial.print("heading: "); Serial.print(gyro.angle); 
////  Serial.print("\ttarget: "); Serial.print(rotPID.target_angle);
////  Serial.print("\tvirt target: "); Serial.print(rotPID.virtual_target);Serial.print("\terror: "); Serial.println(rotPID.getErrorABS());
//
//  updateMotors(dt);
//  uno.setMotorsAngular(rotPID.update(dt, gyro.angle));
//}
