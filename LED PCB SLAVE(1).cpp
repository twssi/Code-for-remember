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


#define DE_PIN 16 // 슬레이브 RS485 모듈의 DE 핀
#define TX_PIN 17 // 슬레이브 RS485 모듈의 TX 핀
#define RX_PIN 19 // 슬레이브 RS485 모듈의 RX 핀
#define MOTOR_PIN 5
#define slaveid 1

HardwareSerial RS485Serial(1);

// WiFi 설정
const char* ssid = "iptime";
const char* password = "12345678";

// NTP 서버 설정
const char* ntpServer = "kr.pool.ntp.org";
const int daylightOffset_sec = 0; 
const long gmtOffset_sec = 32400; 

TaskHandle_t motorcontroltask;
TaskHandle_t receivemotorsettingtask;

struct MotorSetting {
  uint16_t motorPeriod;
  uint16_t motorDuration;
};

MotorSetting currentMotorSetting;

void writeMotorSettingToEEPROM();


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



void receiveMotorSetting(void* parameters) {
  while (1) {
    if(RS485Serial.available() > 0){
      uint8_t reciveId = RS485Serial.read();
      if(reciveId == slaveid){
        if (RS485Serial.available() >= sizeof(MotorSetting)) {
          RS485Serial.readBytes((uint8_t*)&currentMotorSetting, sizeof(MotorSetting));

          // Combining 2byte
          uint16_t motorDuration = (currentMotorSetting.motorDuration >> 8) | (currentMotorSetting.motorDuration << 8);
          uint16_t motorPeriod = (currentMotorSetting.motorPeriod >> 8) | (currentMotorSetting.motorPeriod << 8);

          // 합쳐진 값으로 설정
          currentMotorSetting.motorDuration = motorDuration;
          currentMotorSetting.motorPeriod = motorPeriod;

          Serial.print("Received motor setting from Master: ");
          
          for (int i = 0; i < sizeof(currentMotorSetting); i++) {
            Serial.print(((uint8_t*)&currentMotorSetting)[i], HEX);
            Serial.print(" ");
          }
          
          Serial.println();
          writeMotorSettingToEEPROM(); // EEPROM에 모터 설정 값을 저장
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms마다 RS485 통신을 체크
  }
}



void writeMotorSettingToEEPROM() {
  EEPROM.put(1, );
  EEPROM.put(2, currentMotorSetting.motorPeriod);
  EEPROM.put(3, currentMotorSetting.motorDuration);
  EEPROM.commit();
}



void readMotorSettingFromEEPROM() {
  uint16_t period;
  uint16_t duration;
  EEPROM.get(2, period);
  EEPROM.get(3, duration);
  currentMotorSetting.motorPeriod = period;
  currentMotorSetting.motorDuration = duration;
}



void motorControlTask(void* parameters) {
  while (1) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    int hour = timeinfo.tm_hour;

    if (hour >= 8 && hour < 20) {
      uint8_t eepromdataread = EEPROM.read(1);
      if (eepromdataread == 'M') {
        Serial.println("Motor started.");
        digitalWrite(MOTOR_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(currentMotorSetting.motorDuration));
        digitalWrite(MOTOR_PIN, LOW);
        Serial.println("Motor stopped.");
        vTaskDelay(pdMS_TO_TICKS(currentMotorSetting.motorPeriod - currentMotorSetting.motorDuration));
      } else {
        digitalWrite(MOTOR_PIN, LOW);
      }
    } else {
      digitalWrite(MOTOR_PIN, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1초마다 모터 설정 값을 체크
  }
}


void setup() {
  EEPROM.begin(4); // EEPROM 메모리 공간을 모터 설정 값 저장을 위해 4바이트만 사용합니다.
  Serial.begin(115200);
  RS485Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);

  readMotorSettingFromEEPROM(); // EEPROM에서 모터 설정 값을 읽어옵니다.
  digitalWrite(MOTOR_PIN, LOW);

  xTaskCreate(receiveMotorSetting, "receiveMotorSetting", 2048, NULL, 1, &receivemotorsettingtask);
  xTaskCreate(motorControlTask, "motorControlTask", 2048, NULL, 2, &motorcontroltask);

  setupWiFi();
  setupTime();

}


void loop() {
  // 루프는 아무 작업을 하지 않습니다.
}
