#ifndef WEB_SERIAL_ENABLED
#define WEB_SERIAL_ENABLED  // comment out this line to disable web terminal
#include "WiFi.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "WebSerial.h"

AsyncWebServer server(80);
const char* ssid = "ESP_2-10";
const char* password = "password";
void recvMsg(uint8_t *data, size_t len)
{
//  WebSerial.println("Received Data...");
//  String d = "";
//  for(int i=0; i < len; i++){
//    d += char(data[i]);
//  }
//  WebSerial.println(d);
}
#endif
