#include <Arduino.h>
#include <HardwareSerial.h>

#define RS485_RX_PIN 15   // ESP32 RXD 핀
#define RS485_TX_PIN 16   // ESP32 TXD 핀
#define RS485_DE_PIN 17   // ESP32 DE 핀 (전송/수신 설정)

#define RS485_BAUDRATE 115200    // RS485 통신 속도

HardwareSerial rs485Serial(1);  // ESP32의 UART1 (Serial1) 사용

void sendToSlave(int, const String&);

void setup() {
  Serial.begin(115200);   // 시리얼 모니터를 위한 초기화
  rs485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);


  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, HIGH);  // 초기에는 송신 모드로 설정
}

void loop() {
  // 슬레이브 1에게 데이터 요청
  static String requestData;

  requestData = "ON";
  sendToSlave(1, requestData);
  delay(3000);

  requestData = "OFF";
  sendToSlave(1, requestData);
  delay(3000);

  // 슬레이브 2에게 데이터 요청
  requestData = "ON";
  sendToSlave(2, requestData);
  delay(3000);

  requestData = "OFF";
  sendToSlave(2, requestData);
  delay(3000);
}

void sendToSlave(int slaveID, const String& command) {
  String message = String(slaveID) + ":" + command;
  rs485Serial.write(message.c_str());
  rs485Serial.flush();
}