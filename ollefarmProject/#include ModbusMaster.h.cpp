#include <HardwareSerial.h>
#include <Arduino.h>

#define DE_PIN 4 // 마스터 RS485 모듈의 DE 핀
#define TX_PIN 17 // 마스터 RS485 모듈의 TX 핀
#define RX_PIN 19// 마스터 RS485 모듈의 RX 핀

HardwareSerial RS485Serial(0);






void sendstring();




void sendstring()
{
  RS485Serial.write("true");
  delay(5000);
    
  RS485Serial.write("false");
  delay(5000);

}


void setup() {
  // UART 통신 초기화
  
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);

  
  RS485Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(115200);
  
  // RS485 통신 설정
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, HIGH);  // 송신 모드로 설정

}

void loop() {
  // 마스터에서 슬레이브로 데이터 전송
  digitalWrite(DE_PIN, HIGH);  // 송신 모드로 설정
  sendstring();

}
