#include <Arduino.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <HardwareSerial.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "time.h"

#define DE_PIN 16 // Master RS485 module DE pin
#define TX_PIN 17 // Master RS485 module TX pin
#define RX_PIN 19 // Master RS485 module RX pin

HardwareSerial RS485Serial(1);

// Message to slaves
uint8_t motorMessage[] = {1, 'M', 0, 0,0,0}; // Setup message to slave 1
uint8_t ledMessage[] = {2, 'H'};        // Setup message to slave 2 (In day time)
uint8_t ledMessage2[] = {2, 'L'};       // Setup message to slave 2 

// Setup WiFi 
const char* ssid = "iptime";
const char* password = "12345678";

// Setup NTP 
const char* ntpServer = "kr.pool.ntp.org";
const int daylightOffset_sec = 0; 
const long gmtOffset_sec = 32400; 

TaskHandle_t motorTaskHandle;
TaskHandle_t ledTaskHandle;

void sendMotorSetting(void* parameters);
void sendLedControl(void* parameters);
void setupWiFi();
void setupTime();


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





void sendMotorSetting(void* parameters) {
  while (1) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    int hour = timeinfo.tm_hour;
    if (hour >= 8 && hour < 20) {
      motorMessage[0] = 1; // Slave ID
      motorMessage[1] = 'M';

      uint16_t motorDuration = 1000;
      uint16_t motorPeriod = 2000;
      
      // Save Duration( lower 8bit & upper 8bit)
      motorMessage[2] = motorDuration & 0xFF; // Duration lower 8bit
      motorMessage[3] = (motorDuration >> 8) & 0xFF; // Duration upper 8bit

      // 작동 주기 하위 8비트와 상위 8비트 저장
      motorMessage[4] = motorPeriod & 0xFF; // 작동 주기 하위 8비트
      motorMessage[5] = (motorPeriod >> 8) & 0xFF; // 작동 주기 상위 8비트else {
    }
    else{ 
      motorMessage[2] = 0; // 모터 동작하지 않도록 설정
      motorMessage[3] = 0;
    }

    // 모터 설정 메시지를 슬레이브 1에게 보냄
    RS485Serial.write(motorMessage, sizeof(motorMessage));
    Serial.print("슬레이브 1에게 모터 설정 메시지 전송: ");
    for (int i = 0; i < sizeof(motorMessage); i++) {
      Serial.print(motorMessage[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(10000)); // 10초마다 모터 설정 값을 보냄
  }
}






void sendLedControl(void* parameters) {
  while (1) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    
    int hour = timeinfo.tm_hour;
    
    if (hour >= 8 && hour < 20) {
      // LED 제어 메시지를 슬레이브 2에게 보냄 (메시지는 항상 동일)
      RS485Serial.write(ledMessage, sizeof(ledMessage));
      Serial.println("슬레이브 2에게 LED 제어 메시지 전송: ");
      for (int i = 0; i < sizeof(ledMessage); i++) {
        Serial.print(ledMessage[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      // LED 제어 메시지를 슬레이브 2에게 보냄 (다른 메시지)
      RS485Serial.write(ledMessage2, sizeof(ledMessage2));
      Serial.print("슬레이브 2에게 LED 제어 메시지 전송: ");
      for (int i = 0; i < sizeof(ledMessage2); i++) {
        Serial.print(ledMessage2[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    vTaskDelay(pdMS_TO_TICKS(20000)); // 20초마다 LED 제어 값을 보냄
  }
}

void setup() {
  
  
  EEPROM.begin(24);
  Serial.begin(115200);
  RS485Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, HIGH);

  xTaskCreate(sendMotorSetting, "sendMotorSetting", 2048, NULL, 1, &motorTaskHandle);
  xTaskCreate(sendLedControl, "sendLedControl", 2048, NULL, 1, &ledTaskHandle);
  
  



  setupWiFi();
  setupTime();


}

void loop() {
  // 루프는 아무 작업을
}