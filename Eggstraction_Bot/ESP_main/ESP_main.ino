#include "Gyro.h"
#include "ESP2UNOComm.h"
#include "MovementUtil.h"




#define MULTI_EGG_MODE

//// WebSerial stuff
//#include "WifiSerial.h"
//#include "Debug.h"

// Obj Detection Library Includes
#define MAX_RESOLUTION_VGA
#define MAX_RECURSION_DEPTH 13

//#define WIFI_ENABLED  // comment out this definition to switch onboard processing only

#include "esp32cam.h"
#include "esp32cam/JpegDecoder.h"
#include "esp32cam/apps/ColorBlobDetector.h"

using namespace Eloquent::Esp32cam;

struct CamTarget
{
  int y;
  int cb;
  int cr;
  int tol;
  int minArea;
  int max_height;
  bool enable_grace_period;
  CamTarget(
    int y,
    int cb,
    int cr,
    int tol = 40,
    int minArea = 0,
    bool enable_grace_period = true,
    int max_height = MAX_RESOLUTION_HEIGHT / 8
  ) : y(y),cb(cb), cr(cr), tol(tol), minArea(minArea), max_height(max_height), enable_grace_period(enable_grace_period){}
};
 
CamTarget pink_egg = CamTarget(230, 130, 200, 30, 128);
//CamTarget yellow_egg = CamTarget(255, 250, 190, 40, 128); Previous
CamTarget yellow_egg = CamTarget(250, 255, 180, 35, 128);
CamTarget purple_egg = CamTarget(153, 78, 140, 30, 128);
CamTarget green_egg = CamTarget(84, 226, 128, 50, 128);
//CamTarget blue_egg = CamTarget(37, 163, 255, 30, 128); Previous
CamTarget blue_egg = CamTarget(67, 200, 255, 50, 128); // Wrote this morning
CamTarget red_square = CamTarget(180, 130, 123, 20, 2048, false);

Cam cam;
JpegDecoder decoder;
Applications::ColorBlobDetector egg_detector(0, 0, 0);
Applications::ColorBlobDetector red_detector(0, 0, 0);



// // End of Obj Detection Includes

#ifdef WIFI_ENABLED
  #include "esp32cam/http/ColorBlobDetectorHTTP.h"
  Http::ColorBlobDetectorHTTP http(cam, decoder, detector);
 
  IPAddress local_IP(192,168,4,22);
  IPAddress gateway(192,168,4,9);
  IPAddress subnet(255,255,255,0);
  
  const char *ssid = "AC-ESP32";
  const char *passphrase = "987654321";
#endif

Gyro gyro;
MovementUtil movement(&uno, &gyro);

TaskHandle_t Task1;

// There are two instances of this struct, one for each core. The one
// declared here is gloabl, the Nav core instance is declared at start
// of that task and only updated at the start of each loop.
struct Target
{
  float left, right, top, bottom;
  bool detected;
  float heading;
};
struct SharedVariables
{
  Target egg, red;
  
  bool fresh_data = false;
} core1_variables;

///// FUNCTION DECLARATIONS /////
float calculate_approx_angle(float center_of_blob);
float BlobArea(float left, float right, float top, float bottom);
void navUpdate(SharedVariables& vars, float dt);
void setDetectorTarget(Applications::ColorBlobDetector& detector, CamTarget target);
void loopCore0(void * parameter);
void printBlobBorders();
void runDetector(Applications::ColorBlobDetector& detector, JpegDecoder& decoder, Target& target);
float mag(float val){ return (val >= 0 ? val : -val); }

#define LIGHT_PIN 4

void setup() {
  Serial.begin(115200);
  Serial.println("Start of setup");

  #ifdef WEB_SERIAL_ENABLED
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
    server.begin();
  #endif
  
  cam.aithinker();
  cam.highQuality();
  cam.vga();
  cam.highestSaturation();
  cam.disableAutomaticWhiteBalance();
  cam.disableAutomaticExposureControl();
  cam.disableGainControl();

  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, HIGH);
  
  while (!cam.begin())
    Serial.println(cam.getErrorMessage());

  
  uno.begin(0x55);  //UNCOMMENT FOR RELEASE
  while(!uno.RX_data.start)
  {
    uno.requestData();
    Serial.print("egg: ");
    Serial.println(uno.RX_data.egg_color);
    delay(100);
  }
  uno.requestData();
  
  CamTarget* egg;
  switch(uno.RX_data.egg_color)
  {
    case 0:
      egg = &pink_egg;
      break;
    case 1:
      egg = &yellow_egg;
      break;
    case 2:
      egg = &green_egg;
      break;
    case 3:
      egg = &blue_egg;
      break;
    default:
      Serial.println("Error: INVALID EGG COLOR");
      while(1);
  }
  setDetectorTarget(egg_detector, *egg);
  setDetectorTarget(red_detector, red_square);
  
  gyro.begin();
  gyro.calibrate(100);
  
  #ifdef WIFI_ENABLED // only executes if this is defined (top of file)
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
  #endif
  
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
}

void loop() {
<<<<<<< Updated upstream
  
=======

>>>>>>> Stashed changes
  float heading_at_capture = gyro.angle();
  
//  //COLOR DETECTION
//  #ifdef WIFI_ENABLED
//    digitalWrite(LIGHT_PIN, HIGH);
//    http.handle();
//    digitalWrite(LIGHT_PIN, LOW);
//  #else
    cam.capture();
    decoder.decode(cam);
//  #endif

  // check image for egg, then for red square
  runDetector(egg_detector, decoder, core1_variables.egg);
  core1_variables.egg.heading = gyro.wrap360(core1_variables.egg.heading + heading_at_capture);
  runDetector(red_detector, decoder, core1_variables.red);
  core1_variables.red.heading = gyro.wrap360(core1_variables.red.heading + heading_at_capture);
  
  // let the Nav loop know that data has been collected
  core1_variables.fresh_data = true;
}

void runDetector(Applications::ColorBlobDetector& detector, JpegDecoder& decoder, Target& target)
{
  target.detected = false;
  
  if(!detector.detect(decoder))
    return;
  
  // if border is all 0s OR border is 80,0,60,0
  if((detector.blob.left == 0 && detector.blob.right == 0 && detector.blob.top == 0 && detector.blob.bottom == 0) ||
    (detector.blob.left == 80 && detector.blob.right == 0 && detector.blob.top == 60 && detector.blob.bottom == 0))
  {
    return;
  }

  // runs if target was detected
  target.detected = true;
  target.left = detector.blob.left;
  target.right = detector.blob.right;
  target.top = detector.blob.top;
  target.bottom = detector.blob.bottom;
  
  float center = (target.right + target.left)/2;
  target.heading = calculate_approx_angle(center);
  
  return;
}

void loopCore0(void * parameter)
{
  SharedVariables core0_variables;
  while(1)
  {
    // record delta time since last loop
    static long prev_micros = 0;
    long time_stamp = micros();
    float dt = (time_stamp - prev_micros) / 1000000.0;
    prev_micros = time_stamp;

    // only copy ObjDet data at the start of loop and if data is not stale
    core0_variables.fresh_data = false; // internal copy
    if(core1_variables.fresh_data)
    {
      core0_variables = core1_variables;
      core1_variables.fresh_data = false;
    }

    // collect gyro data and integrate to maintain heading info
    for(int i = 0; i < 10; i++)
    {
      gyro.update();
      delayMicroseconds(2500);
    }

    uno.requestData();
    navUpdate(core0_variables, dt);
    uno.sendData();
    
  }
}

enum State
{
  start,
  scanning,
  searching,
  seeking,
  initiate_grab,
  grabbing,
  looking_for_red,
  egg_reacquisition,
  returning,
  border_hit,
  border_hit_while_returning,
  initiate_release,
  releasing,
  halt,
  multi_egg_reset
};

State state = searching;

bool scan_dir = false;
bool tracking_square = false;
float angle_covered = 0;
unsigned long timer = -1;
float target_heading = 0;
bool checkBorder()
{
  if((uno.getLeftIRSensor() == IRColor::BLACK || uno.getRightIRSensor() == IRColor::BLACK))
  {
    if(uno.getRightIRSensor() == IRColor::BLACK)
      scan_dir = true;  // turning CCW
    else
      scan_dir = false; // turning CW
    
    return true;
  }
  return false;
}
void navUpdate(SharedVariables& vars, float dt)
{
    
  static State prev_state = scanning;  
  static bool first_loop = false;
  if(state != prev_state)
  {
    prev_state = state;
    movement.cutMotors();
    timer = millis();
    first_loop = true;
  }
  switch(state) // main state machine
  {
    case start:
    {
      if(first_loop)
      {
        movement.setRotTarget();
        first_loop = false;
      }
      
      if(millis() - timer > 1500)
      {
        movement.cutMotors();
        state = scanning; 
      }
      else
      {
        movement.setForward(0.9, dt);
        movement.holdHeading(dt);
      }
      break;
    }
    case border_hit:
    {
      if(first_loop)
      {
        first_loop = false;
        target_heading = gyro.wrap360(gyro.angle() + (scan_dir ? 1.0 : -1.0) * 100);
      }
      
      if(mag(target_heading - gyro.angle()) < 2.0)
      {
        state = searching;
        movement.cutMotors();
      }
      else  // turn away from the border at 60 deg/s
      {
        movement.goToHeading(target_heading, 90, dt);
      }

      if(vars.egg.detected && !vars.red.detected) // egg detected
      {
        movement.cutMotors();
        state = seeking;
      }
      
      break;
    }
    case border_hit_while_returning:
    {
      if(first_loop)
      {
        first_loop = false;
        target_heading = gyro.wrap360(gyro.angle() + (scan_dir ? 1.0 : -1.0) * 10);
      }
      
      const float reverse_time = 500;
      if(mag(target_heading - gyro.angle()) < 2.0)
      {
        movement.cutMotors();
        state = returning;
      }
      else if(millis() - timer > reverse_time)
      {
        movement.cutMotors();
        movement.goToHeading(target_heading, 60, dt);
      }
      else 
      {
        movement.setForward(-0.3, dt);
      }
      break;
    }
    case scanning:  // turning in place looking for the desired egg color
    {
      //const float scan_angle = 360;
      const float scan_angle = 405; //Scan 360 + 45
      static float starting_displacement = 0;
      
      if(first_loop)
      {
        uno.TX_data.gripper_signal = false;
        first_loop = false;
        starting_displacement = gyro.displacement;
      }
      
      if(mag(gyro.displacement - starting_displacement) >= scan_angle)
      {        
        movement.setRotTarget();
        movement.cutMotors();
        state = searching;
      }
      else if(vars.egg.detected && !vars.red.detected) // egg detected
      {
        movement.cutMotors();
        state = seeking;
      }

      movement.scan(90 * (scan_dir ? 1.0 : -1.0), dt);  // -spd to scan clockwise
      
      if(checkBorder())
        state = border_hit;
        
      break;
    }
    case searching:
    {
      const int move_time = 3000;
      if(first_loop)
      {
        first_loop = false;
        movement.setRotTarget();
      }
      
      if(millis() - timer > move_time)
      {
        state = scanning;
      }
      else if(vars.egg.detected) // egg detected
      {
        state = seeking;
        break;
      }
      movement.holdHeading(dt);
      movement.setForward(0.8, dt);

      if(checkBorder())
        state = border_hit;
      
      break;
    }
    case seeking: // tracking and moving toward egg, reducing speed as it approaches
    {
      const int grace_period = 500; //ms

      if(uno.getLeftIRSensor() == IRColor::RED && uno.getRightIRSensor() == IRColor::RED) // the egg will never be on the red square, get off of it
      {
        state = start;
      }
      else if(movement.seek(vars.egg.heading, vars.egg.top, dt)) // if you're at the egg, start grabbing
      {
        state = initiate_grab;
      }
      else if(!vars.egg.detected && (millis() - timer) > grace_period) // go back to scanning if you lost the egg
      {
        scan_dir = (gyro.relativeAngle(vars.egg.heading) > 0);  // scan direction is same as last-known target direction
        state = scanning;
      }
      else if(vars.egg.detected)
      {
        timer = millis();
      }

      if(checkBorder())
        state = border_hit;
        
      break;
    }
    case initiate_grab: // transition state to wait for gripper on uno side to confirm start-of-grab
    {
      if(uno.RX_data.gripper_active && !uno.TX_data.gripper_signal) // wait if gripper is currently closing
        break;
        
      uno.TX_data.gripper_signal = true;
      if(uno.RX_data.gripper_active)  // wait for grab to initiate on uno side, then go to grabbing
        state = grabbing;
      
      break;
    }
    case grabbing:  // loitering state while gripper closes
    {
      if(!uno.RX_data.gripper_active) // start returning if grip finished successfully
      {
        state = looking_for_red;
      }
      break;
    }
    case looking_for_red: // basically scanning again, but for the red square this time
    {
      movement.scan(60, dt);
      int egg_grace_period = 500;  //ms
      if(!vars.egg.detected && (millis() - timer) > egg_grace_period)
      {
        uno.TX_data.gripper_signal = false;
        state = scanning;
        break;
      }
      else if(vars.egg.detected)
      {
        timer = millis();
      }
      
      if(vars.red.detected)
      {
        movement.cutMotors();
        movement.setRotTarget();
        state = returning;
      }
      if(checkBorder())
        state = border_hit_while_returning;
        
      break;
    }
    case returning: // you've got the egg and found the red square, just head in that direction until you reach it
    {
      float red, green, blue;
      delay(60);  // takes 50ms to read
      tcs.getRGB(&red, &green, &blue);

      const int egg_grace_period = 500;  //ms
      const int red_grace_period = 500;
      static unsigned long egg_timer = 0;
      static unsigned long red_timer = 0;
      if(first_loop)
      {
        first_loop = false;
        egg_timer = red_timer = millis();
      }
      
      if(!vars.egg.detected && (millis() - egg_timer) > egg_grace_period)
      {
        uno.TX_data.gripper_signal = false;
        state = scanning;
        break;
      }
      else if(vars.egg.detected)
      {
        egg_timer = millis();
      }

      if(!vars.red.detected && (millis() - red_timer) > red_grace_period)
      {
        movement.cutMotors();
        state = looking_for_red;
        break;        
      }
      else if(vars.red.detected)
      {
        red_timer = millis();
      }
      
      movement.seek(vars.red.heading, 0, dt);
      movement.setForward(0.8, dt);
      
      if(uno.getLeftIRSensor() == IRColor::RED && uno.getRightIRSensor() == IRColor::RED)
      {
        movement.cutMotors();
        state = initiate_release;
      }
      else if(uno.getLeftIRSensor() == IRColor::BLACK || uno.getRightIRSensor() == IRColor::BLACK)
      {
        movement.cutMotors();
        if(uno.getRightIRSensor() == IRColor::BLACK)
          scan_dir = true;  // turning CCW
        else
          scan_dir = false; // turning CW 
        
        state = border_hit_while_returning;
      }
      if(checkBorder())
        state = border_hit_while_returning; 

      break;
    }
    case initiate_release:
    {
      uno.TX_data.gripper_signal = false;
      movement.cutMotors();
      if(uno.RX_data.gripper_active)
      {
        state = releasing;
      }
      break;
    }
    case releasing:
    {
      if(uno.RX_data.gripper_active)
      {
        timer = millis();
      }
      else if((millis() - timer) < 1000)
      {
        uno.setMotorsAngular(0);
        movement.setForward(-0.2, dt);
      }
      else
      {
        movement.cutMotors();
        state = halt;
      }
      break;
    }
    case halt:  // open the gripper, and do nothing
    {
      movement.cutMotors();
      
      #ifdef MULTI_EGG_MODE
        state = multi_egg_reset;
      #endif
      
      break;
    }
    case multi_egg_reset:
    {
      if(first_loop)
      {
        first_loop = false;
        target_heading = gyro.wrap360(gyro.angle() + 180);
      }
      
      if(mag(target_heading - gyro.angle()) < 2.0)
      {
        state = searching;
        movement.cutMotors();
      }
      else
      {
        movement.goToHeading(target_heading, 90, dt);
      }
      break;
    }
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
  //debugPrint("blob data l,r,t,b: ");
  //debugPrint(detector.blob.left); //debugPrint(' ');
  //debugPrint(detector.blob.right); //debugPrint(' ');
  //debugPrint(detector.blob.top); //debugPrint(' ');
  //debugPrintln(detector.blob.bottom);
}

void setDetectorTarget(Applications::ColorBlobDetector& detector, CamTarget target)
{
  detector.set("y", target.y);
  detector.set("cb", target.cb);
  detector.set("cr", target.cr);
  detector.tollerate(target.tol);
  detector.setMinArea(target.minArea);
}
