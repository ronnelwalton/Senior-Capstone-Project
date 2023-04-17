/*********
  Created by Ronnel Walton
  Uses Async WebServer instead of Default WebServer
  Serial communication has also been integrated into this version
*********/

#include "index.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SoftwareSerial.h>

#define RXp2 13
#define TXp2 14

//Site to visit for UI
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// Replace with your network credentials
const char* ssid = "65 ACT UI";
const char* password = "123456789";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Stored Distance Measurement in cm
float prevDist = 0;

// Flag for website Measurement
int flag = 0;

// Return the stored distance measurement
float retrieveDist(){
  return prevDist;
}


void setup(){
  // Serial port for debugging purposes
  //Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.mode(WIFI_AP); //Access Point mode
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  
  // Print ESP Local IP Address

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", UI_page);
  });


  // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
  server.on("/readDistance", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String disVal = "";
    float d = retrieveDist();
    if (d == 1) {
      request->send(200, "text/plain", "Merging...");
    } else if (d != 0){
      disVal = "Following distance: ";
      if (flag == -1) {
        flag = 0;
        disVal = disVal + ">50 cm";
      } else {
        String distStr = String(((int)(d+0.5)));
        disVal = disVal + distStr + " cm";      
      }
      request->send(200, "text/plain", disVal);
    } else {
      request->send(200, "text/plain", "Parked, No cars detected");
    }
  });
  

  // Start server
  server.begin();
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
}

// Retrieve Distance Measurement from Arduino Uno
void loop() {
  if (Serial2.available() > 0){ // Check if theres any bytes coming from Arduino Uno
    String receivedMsg = Serial2.readStringUntil('S');
    float distVal = receivedMsg.toFloat();
    if (distVal == -1) {
      flag = -1;
    } else {
      prevDist = distVal;
      flag = 0;
    }
  }
} 
