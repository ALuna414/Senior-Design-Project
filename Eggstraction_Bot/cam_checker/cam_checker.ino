#include "Gyro.h"
#include "ESP2UNOComm.h"
#include "RotationPID.h"

// Obj Detection Library Includes
#define MAX_RESOLUTION_VGA
#define MAX_RECURSION_DEPTH 13

#include "esp32cam.h"
#include "esp32cam/JpegDecoder.h"
#include "esp32cam/apps/ColorBlobDetector.h"
#include "esp32cam/http/ColorBlobDetectorHTTP.h"
#include "WiFi.h"

using namespace Eloquent::Esp32cam;

struct Color
{
  int y;
  int cb;
  int cr;
  int tol;
  int minArea;
  Color(int y, int cb, int cr, int tol = 40, int minArea = 0) : y(y), cb(cb), cr(cr), tol(tol), minArea(minArea){}
};

Color pink_egg = Color(250, 140, 200, 50, 0);
Color red_square = Color(180, 150, 140, 10, 1024);

Cam cam;
JpegDecoder decoder;
Applications::ColorBlobDetector detector(250, 140, 200); // will configure from the URL
Http::ColorBlobDetectorHTTP http(cam, decoder, detector);
// // End of Obj Detection Includes
IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

const char *ssid = "AC-ESP32";
const char *passphrase = "987654321";

Gyro gyro;
RotationPID rotPID(0.015, 0.01, 0.00); // (P, I, D, deadzone deg)
float PID_rate_P = 0.01;
float PID_rate_I = 0.005;

TaskHandle_t Task1;

struct SharedVariables
{
  float target_bearing = 0;
  bool target_detected = false;
  bool fresh_data = false;
  float heading_at_capture = 0;
} core1_variables;

///// FUNCTION DECLARATIONS /////
float calculate_approx_angle(float center_of_blob);
float BlobArea(float left, float right, float top, float bottom);
void navUpdate(SharedVariables& vars, float dt);
void scan(float search_spd, float dt);
bool seek(SharedVariables& vars, float dt);
void updatePID(float dt);
void track(SharedVariables& vars, float dt);
void setDetectorColor(Color color);
void loopCore0(void * parameter);
void printBlobBorders();
void updateMotors(float target_spd, float dt);

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

  //Color Blob Parameters
  setDetectorColor(pink_egg);

  while (!cam.begin())
        Serial.println(cam.getErrorMessage());
  
//  uno.begin(0x55);
//  gyro.begin();
//  gyro.calibrate(100);

  rotPID.setPIDRate(PID_rate_P, PID_rate_I, 0);

  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid,passphrase) ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  while (!http.begin())
    Serial.println(http.getErrorMessage());

  cam.mDNS("esp32cam");
  Serial.println(http.getWelcomeMessage());

  xTaskCreatePinnedToCore(
    loopCore0,
    "loopCore0",
    10000,
    NULL,
    0,
    &Task1,
    0
  );

  delay(500);
  pinMode(4, OUTPUT);
}

void loop() {

  //COLOR DETECTION
  core1_variables.heading_at_capture = gyro.angle;
  digitalWrite(4, HIGH);
  http.handle();
  digitalWrite(4, LOW);
  
  float center_of_blob = (detector.blob.right + detector.blob.left)/2;
  float blob_area = BlobArea(detector.blob.left, detector.blob.right, detector.blob.top, detector.blob.bottom);


  ///// GRACE PERIOD /////
  /*
   * if the camera loses the egg, the system assumes it is still in the same 
   * position for a specified grace period. After the grace period is
   * up target_detected is set false. If the egg is re-discovered during the
   * grace period then the egg_angle is immediately updated. egg_angle is 
   * assumed to be undefined when target_detected is false.
   */
//  static unsigned long egg_timer = 0;
//  static bool grace_period = false;
//  const int grace_period_duration = 500;  //ms

  
  if((detector.blob.left == 0 && detector.blob.right == 0 && detector.blob.top == 0 && detector.blob.bottom == 0) ||
    (detector.blob.left == 80 && detector.blob.right == 0 && detector.blob.top == 60 && detector.blob.bottom == 0))
  { 
    core1_variables.target_detected = false;
//    if(core1_variables.target_detected)
//    {
//      if(!grace_period)
//      {
//        grace_period = true;
//        egg_timer = millis();
//      }
//      else if(millis() > egg_timer + grace_period_duration)
//      {
//        core1_variables.target_detected = false;
//        grace_period = false;
//      }
//    }
  }
//  else
//  {
//    core1_variables.target_detected = true;
//    grace_period = false;
//  }
  else
  {
    core1_variables.target_detected = true;
  }
  if(core1_variables.target_detected)// && !grace_period)
  {
    core1_variables.target_bearing = calculate_approx_angle(center_of_blob); 
  }
  core1_variables.fresh_data = true;
}

void loopCore0(void * parameter)
{
  SharedVariables core0_variables;
  while(1)
  {    
//    static long prev_micros = 0;
//    float dt = (micros() - prev_micros) / 1000000.0;
//    prev_micros = micros();
//
//    if(core1_variables.fresh_data)
//    {
//      core0_variables = core1_variables;
//      core1_variables.fresh_data = false;
//    }
//    
//    for(int i = 0; i < 10; i++)
//    {
//      gyro.update();
//      delayMicroseconds(2500);
//    }
//    
////    uno.requestData();
////    navUpdate(core0_variables, dt);
////    uno.sendData();
//    
//    Serial.println(gyro.angle);
//    
////    void printBlobBorders();
////    delay(100);
//    core0_variables.fresh_data = false;
  }
}

enum State
{
  scanning,
  searching,
  seeking,
  initiate_grab,
  grabbing,
  looking_for_red,
  returning,
  halt
};

State state = scanning;
bool scan_cw = false;
bool tracking_square = false;
void navUpdate(SharedVariables& vars, float dt)
{
  Serial.print("state is: "); Serial.println(state);
  switch(state)
  {
    case scanning:
    {
      Serial.println("scanning");

      scan(45 * (scan_cw ? -1 : 1), dt);  // -spd to scan clockwise
      
      if(vars.target_detected && vars.fresh_data)
      {
        uno.setMotorsAngular(0);
        updateMotors(0, dt);
        state = seeking;
      }
      break;
    }
    case seeking:
    {
      Serial.println("seeking");
      if(!vars.target_detected) // go back to scanning if you lost the egg
      {
        scan_cw = (vars.target_bearing < 0);  // scan direction is same as last-known target direction
        state = scanning;
        break;
      }

      if(seek(vars, dt)) // if you're at the egg, start grabbing
      {
        state = initiate_grab;
      }
      break;
    }
    case initiate_grab:
    {
      Serial.println("initiate_grab");
      
      uno.TX_data.gripper_signal = true;
      if(uno.RX_data.gripper_active)  // wait for grab to initiate on uno side, then go to grabbing
        state = grabbing;
      updateMotors(0, dt);
      uno.setMotorsAngular(0);
      break;
    }
    case grabbing:
    {
      Serial.println("grabbing");

      if(!vars.target_detected) // if you lost the egg, go back to scanning
      {
        state = scanning;
        uno.TX_data.gripper_signal = false;
        break;
      }
      
      if(!uno.RX_data.gripper_active) // start returning if grip finished successfully
      {
        setDetectorColor(red_square);
        state = looking_for_red;
      }
      
      uno.setMotorsAngular(0);
      updateMotors(0, dt);
      break;
    }
    case looking_for_red:
    {
      scan(45, dt);
      if(vars.target_detected && vars.fresh_data)
      {
        setDetectorColor(pink_egg);
        rotPID.setTargetAngle(vars.heading_at_capture + vars.target_bearing);
        state = returning;
      }
      break;
    }
    case returning:
    {
      Serial.println("returning");

      if(!vars.target_detected && vars.fresh_data)
      {
        state = scanning;
      }
      
      updateMotors(0.6, dt);
      uno.setMotorsAngular(rotPID.update(dt, &gyro));

      if(uno.getLeftIRSensor() == ESP2UNOComm::IRColor::RED && uno.getRightIRSensor() == ESP2UNOComm::IRColor::RED)
      {
        updateMotors(0, dt);
        uno.setMotorsAngular(0);
        state = halt;
      }

      break;
    }
    case halt:
    {
      Serial.println("halt");
      uno.setMotorsAngular(0);
      updateMotors(0, dt);
      uno.TX_data.gripper_signal = false;
      break;
    }
  }
}

void scan(float search_spd, float dt)
{
  rotPID.setMode(RotationPID::hold_rate);
  rotPID.setTargetAngle(search_spd);
  
  updateMotors(0, dt);  
  uno.setMotorsAngular(rotPID.update(dt, &gyro));
}

bool seek(SharedVariables& vars, float dt)
{
  rotPID.setMode(RotationPID::hold_angle);
  track(vars, dt);

  bool at_target = false;
  float target_speed = 0;
  float mag = (vars.target_bearing >= 0) ? vars.target_bearing : -vars.target_bearing;
  if(mag < 10)
  {
    if(detector.blob.bottom < 30)
      target_speed = 0.6;
    else if(detector.blob.bottom < 55)
      target_speed = 0.10;
    else
      at_target = true;
  }
  Serial.print("blob bottom: "); Serial.println(detector.blob.bottom);
  updateMotors(target_speed, dt);
  uno.setMotorsAngular(rotPID.update(dt, &gyro));
  return at_target;
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

void track(SharedVariables& vars, float dt)
{
  static float prev_bearing = 0;
  if(prev_bearing != vars.target_bearing)
  {
    rotPID.setTargetAngle(gyro.wrap360(vars.heading_at_capture + vars.target_bearing));
    prev_bearing = vars.target_bearing;
  }
}

float calculate_approx_angle(float center_of_blob)
{
  const float manual_correction_factor = 0.95;
  float value = ((40 - center_of_blob) * 33 / 40) * manual_correction_factor;
  return value;
}

// (entire screen), it is default selection and an egg isn't actually being seen
float BlobArea(float left, float right, float top, float bottom)
{
  return((right - left) * (bottom - top));
}

void printBlobBorders()
{
  Serial.print("blob data l,r,t,b: ");
  Serial.print(detector.blob.left); Serial.print(' ');
  Serial.print(detector.blob.right); Serial.print(' ');
  Serial.print(detector.blob.top); Serial.print(' ');
  Serial.println(detector.blob.bottom);
}

void setDetectorColor(Color color)
{
  detector.set("y", color.y);
  detector.set("cb", color.cb);
  detector.set("cr", color.cr);
  detector.tollerate(color.tol);
  detector.setMinArea(color.minArea);

  core1_variables.fresh_data = false;
}
