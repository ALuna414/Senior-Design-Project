#define MAX_RESOLUTION_VGA
#define MAX_RECURSION_DEPTH 13

#include "esp32cam.h"
#include "esp32cam/JpegDecoder.h"
#include "esp32cam/apps/ColorBlobDetector.h"
#include "esp32cam/http/ColorBlobDetectorHTTP.h"
#include <WiFi.h>

using namespace Eloquent::Esp32cam;

Cam cam;
JpegDecoder decoder;
Applications::ColorBlobDetector detector(220, 112, 115); // will configure from the URL
Http::ColorBlobDetectorHTTP http(cam, decoder, detector);

IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

const char *ssid = "AC-ESP32";
const char *passphrase = "987654321";

float calculate_approx_angle(float center_of_blob)
{
  float percent_away;
  //To the LEFT
  if (center_of_blob <= 40.0)
  {
    percent_away = 1 - (center_of_blob / 40.0);
    return(percent_away * -33);
  }
  //To the RIGHT
  else
  {
    center_of_blob -= 40.0;
    percent_away = center_of_blob / 40.0;
    return(percent_away * 33);
  }
}

// Method to determine if a blob is detected. Does this by calcuating area of blob detected. If very large
// (entire screen), it is default selection and an egg isn't actually being seen
float BlobArea(float left, float right, float top, float bottom)
{
  return((right - left) * (bottom - top));
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Init");

    cam.aithinker();
    cam.highQuality();
    cam.vga();
    cam.highestSaturation();
    cam.disableAutomaticWhiteBalance();
    cam.disableAutomaticExposureControl();
    cam.disableGainControl();

    //Color Blob Parameters
    detector.tollerate(40);
    detector.setMinArea(7 * 7);

    while (!cam.begin())
        Serial.println(cam.getErrorMessage());

    /* while (!cam.connect("Carson's Hot Spot Connection", "14121311Aa-"))
        Serial.println(cam.getErrorMessage()); */

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
}


void loop() {
    http.handle();
    
        /*Serial.print("Blob detected from top-left ");
        Serial.print(detector.blob.top);
        Serial.print(", ");
        Serial.print(detector.blob.left);
        Serial.print(" to bottom-right ");
        Serial.print(detector.blob.bottom);
        Serial.print(", ");
        Serial.println(detector.blob.right);
        Serial.print("Blob detection run in ");
        Serial.print(detector.getExecutionTimeInMillis());
        Serial.println("ms");*/
        //TOP-BOTTOM ranges 0-60
        //LEFT-RIGHT ranges 0-80

        float center_of_blob = ((detector.blob.right - detector.blob.left)/2) + detector.blob.left;
        if(BlobArea(detector.blob.left, detector.blob.right, detector.blob.top, detector.blob.bottom) < 4000) //4000 is 83 % of screen space...
        {
          //Center Point
          Serial.print("~~~ Center of Blob: ");
          Serial.println(center_of_blob);

          //Area of Rectangle
          Serial.print("--- AREA: ");
          Serial.println(BlobArea(detector.blob.left, detector.blob.right, detector.blob.top, detector.blob.bottom));

          //Angle
          Serial.print("||| ANGLE: ");
          Serial.print(calculate_approx_angle(center_of_blob));
          Serial.println("|||");
          delay(1000);
        }

    else 
    {
        Serial.println("No BLOB Detected ... ... ...");
    }
}