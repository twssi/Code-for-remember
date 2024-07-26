#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>


extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

//replace with your network credentials
#define WIFI_SSID "K-BD"
#define WIFI_PASSWORD "12345678"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 200, 136)
#define MQTT_PORT 1883

//MQTT Topic
#define MQTT_PUB_Output "esp32/OutputControl"

const int PushButton = 15;
int Push_button_state;
bool Push_button_Prv_state = false;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
       

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); 
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(PushButton, INPUT);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

void loop() {
    Push_button_state = digitalRead(PushButton);
    if (Push_button_state == HIGH && Push_button_Prv_state == false)
    {
    // Publish an MQTT message on topic esp32/OutputControl
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_Output, 1, true, "ON");                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_Output, packetIdPub1);
    Serial.println(" Message: ON");
    Push_button_Prv_state = true;
    }
    else if (Push_button_state == LOW && Push_button_Prv_state == true)
    {
    // Publish an MQTT message on topic esp32/OutputControl
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_Output, 1, true, "OFF");                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_Output, packetIdPub2);
    Serial.println(" Message: OFF");
    Push_button_Prv_state = false;
    }
  delay(1000);
}