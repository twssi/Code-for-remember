#include <Arduino.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <HardwareSerial.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "FS.h"
#include "time.h"


#define EEPROM_SIZE 24  
#define DE_PIN 4    // 슬레이브 RS485 모듈의 DE 핀
#define TX_PIN 17   // 슬레이브 RS485 모듈의 TX 핀
#define RX_PIN 19   // 슬레이브 RS485 모듈의 RX 핀
#define slaveid 2

const int ledPin = 16;

HardwareSerial Rs485Serial(1); // hardware setup


// Task setup
TaskHandle_t ledTaskHandle;
TaskHandle_t receivemessage;

// Wi-Fi setup
const char* ssid = "iptime";
const char* password = "12345678";


// NTP setup
const char* ntpServer = "kr.pool.ntp.org";
const int daylightOffset_sec = 0; 
const long gmtOffset_sec = 32400; 



void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}


void setupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}



void receiveMessage(void *pvParamters){
  while(1){
    if(Rs485Serial.available()){
      char receivedata = (char)Rs485Serial.read();
      if(receivedata == slaveid){
        char receivedata = (char)Rs485Serial.read();

        if(receivedata == 'H'){
          EEPROM.write(0, 1);
          EEPROM.commit();
        }     
        
        if(receivedata == 'L'){
          EEPROM.write(0, 0);
          EEPROM.commit();
        }
      }
    }   
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}



void LEDTask(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    struct tm timeinfo;
    
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    int hour = timeinfo.tm_hour;
    
    if(hour >= 8 && hour <20){

      uint8_t eepromdataread = EEPROM.read(0);
      
      if (eepromdataread == 1) {
        digitalWrite(ledPin, HIGH);
        Serial.println("Running(High)");  
      } else {
        digitalWrite(ledPin, LOW);
        Serial.println("Running(Low)");
      }
      }else{
        digitalWrite(ledPin, LOW);
      }
    }
}




void setup() {


  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  Rs485Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  pinMode(ledPin, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  
  digitalWrite(ledPin, LOW);
  digitalWrite(DE_PIN, LOW);
  
  xTaskCreate(receiveMessage, "receiveMessage", 2048, NULL, 1, &receivemessage);
  xTaskCreate(LEDTask, "LEDTask", 2048, NULL, 1, &ledTaskHandle);
  
  
  setupWiFi();
  setupTime();

}

void loop() {
  // 루프는 아무 작업을 수행하지 않습니다.
}
