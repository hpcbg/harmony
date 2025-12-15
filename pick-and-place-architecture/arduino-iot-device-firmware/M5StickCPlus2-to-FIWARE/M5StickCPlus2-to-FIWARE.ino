#include <M5StickCPlus2.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "porthub.h"
#include "config.h"

PortHub porthub;
uint8_t HUB_ADDR[6] = { HUB1_ADDR, HUB2_ADDR, HUB3_ADDR, HUB4_ADDR, HUB5_ADDR, HUB6_ADDR };
int angleValue = 0;
int lastAngleValue = 0;
bool buttonBlue = false;
bool buttonRed = false;
bool lastButtonBlue = false;
bool lastButtonRed = false;

unsigned long lastMsg = 0;
const int threshold = 10;    // angle threshold

void setup() {
  M5.begin();
  Serial.begin(115200);  // For debugging
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("M5+PbHUB+FIWARE");

  Serial.println("Initializing PbHUB...");
  porthub.begin();

  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }

  delay(1000);
  
  setupWiFi();
  
  createEntity();
  
  delay(2000);
}

void setupWiFi() {
  delay(10);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("WiFi...");
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }
  
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.println("WiFi OK!");
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println(WiFi.localIP());
  M5.Lcd.setTextSize(2);
  delay(2000);
}

void createEntity() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    String url = String(fiware_server) + "/v2/entities";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Fiware-Service", fiware_service);
    http.addHeader("Fiware-ServicePath", fiware_servicepath);
    
    StaticJsonDocument<512> doc;
    doc["id"] = device_id;
    doc["type"] = "SensorDevice";
    
    JsonObject angleAttr = doc.createNestedObject("angle");
    angleAttr["type"] = "Integer";
    angleAttr["value"] = 0;
    
    JsonObject blueBtn = doc.createNestedObject("buttonBlue");
    blueBtn["type"] = "Boolean";
    blueBtn["value"] = false;
    
    JsonObject redBtn = doc.createNestedObject("buttonRed");
    redBtn["type"] = "Boolean";
    redBtn["value"] = false;
    
    String payload;
    serializeJson(doc, payload);
    
    int httpCode = http.POST(payload);
    
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.setTextSize(1);
    
    if (httpCode == 201 || httpCode == 422) {
      M5.Lcd.println("Fiware Entity OK!");
    } else {
      M5.Lcd.print("Error: ");
      M5.Lcd.println(httpCode);
    }
    
    M5.Lcd.setTextSize(2);
    http.end();
    delay(1000);
  }
}

void readSensors() {
  angleValue = porthub.hub_a_read_value(HUB_ADDR[5]);
  angleValue = map(angleValue, 0, 4095, 0, 360);

  buttonBlue = !porthub.hub_d_read_value_A(HUB_ADDR[0]);  // Bit 0 (inverted logic)
  buttonRed =  !porthub.hub_d_read_value_B(HUB_ADDR[0]);  // Bit 1 (inverted logic)
}

void updateEntity() {
  if (WiFi.status() == WL_CONNECTED) {
    readSensors();
    
    bool angleChanged = abs(angleValue - lastAngleValue) > threshold;
    bool buttonChanged = (buttonBlue != lastButtonBlue) || (buttonRed != lastButtonRed);
    
    if (angleChanged || buttonChanged) {
      HTTPClient http;
      
      String url = String(fiware_server) + "/v2/entities/" + device_id + "/attrs";
      http.begin(url);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("Fiware-Service", fiware_service);
      http.addHeader("Fiware-ServicePath", fiware_servicepath);
      
      StaticJsonDocument<256> doc;
      
      JsonObject angleAttr = doc.createNestedObject("angle");
      angleAttr["type"] = "Integer";
      angleAttr["value"] = angleValue;
      
      JsonObject blueBtn = doc.createNestedObject("buttonBlue");
      blueBtn["type"] = "Boolean";
      blueBtn["value"] = buttonBlue;
      
      JsonObject redBtn = doc.createNestedObject("buttonRed");
      redBtn["type"] = "Boolean";
      redBtn["value"] = buttonRed;
      
      String payload;
      serializeJson(doc, payload);
      
      int httpCode = http.PATCH(payload);
      
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 5);
      M5.Lcd.setTextSize(1);
      M5.Lcd.println("Fiware Update");
      
      M5.Lcd.setCursor(0, 25);
      M5.Lcd.setTextSize(2);
      M5.Lcd.print("Angle: ");
      M5.Lcd.print(angleValue);
      M5.Lcd.println(" deg");
      
      M5.Lcd.setCursor(0, 45);
      M5.Lcd.print("Blue: ");
      M5.Lcd.println(buttonBlue ? "ON" : "OFF");
      
      M5.Lcd.setCursor(0, 65);
      M5.Lcd.print("Red:  ");
      M5.Lcd.println(buttonRed ? "ON" : "OFF");
      
      M5.Lcd.setCursor(0, 85);
      M5.Lcd.setTextSize(1);
      if (httpCode == 204) {
        M5.Lcd.println("Status: OK");
      } else {
        M5.Lcd.print("Error: ");
        M5.Lcd.println(httpCode);
      }
      
      lastAngleValue = angleValue;
      lastButtonBlue = buttonBlue;
      lastButtonRed = buttonRed;
      
      http.end();
    }
  }
}

void loop() {
  M5.update();
  
  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    updateEntity();
  }
  
  delay(10);
}