#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>



#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/led"
#define AWS_IOT_PUBLISH_TOPIC "esp32/led"

#define lamp1 17
#define lamp2 16
#define lamp3 15

const u_int8_t ledPins[3] = {lamp1, lamp2, lamp3};


WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void messageHandler(char* topic, byte* payload, unsigned int length);
void publishMessage();


void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IOT");
 
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}
 


void publishMessage()
{
  StaticJsonDocument<500> doc;
  doc["led1"] = digitalRead(lamp1);
  doc["led2"] = digitalRead(lamp2);
  doc["led3"] = digitalRead(lamp3);
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}




void messageHandler(char* topic, byte* payload, unsigned int length)
{
  
  Serial.println("**************");
  Serial.print("incoming: ");
  Serial.println(topic);
  
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }


  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, messageTemp);

  if (String(topic) == AWS_IOT_SUBSCRIBE_TOPIC && error == DeserializationError::Ok) {
    JsonObject root = doc.as<JsonObject>();

    // Iterate through the LED keys
    for (int i = 1; i <= 3; i++) {
      String ledKey = "led" + String(i);
      
      if (root.containsKey(ledKey)) {
        const char* ledState = root[ledKey].as<const char*>();

        Serial.print("Changing output of LED ");
        Serial.print(i);
        Serial.print(" to ");
        
        if (strcmp(ledState, "on") == 0) {
          Serial.println("on");
          digitalWrite(ledPins[i - 1], HIGH); // Turn on the corresponding LED
        } else if (strcmp(ledState, "off") == 0) {
          Serial.println("off");
          digitalWrite(ledPins[i - 1], LOW); // Turn off the corresponding LED
        }

      }
    
    }

  }
  Serial.println();
}
 
 
void setup()
{
  Serial.begin(115200);
  connectAWS();
  

  pinMode(lamp1, OUTPUT);
  pinMode(lamp2, OUTPUT);
  pinMode(lamp3, OUTPUT);
  
  digitalWrite(lamp1, LOW);
  digitalWrite(lamp2, LOW);
  digitalWrite(lamp3, LOW);
  
}
 
void loop()
{
  client.loop();
  delay(1000);
}