#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>


#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
 
#define lamp1 17
#define lamp2 16
#define lamp3 15

//DHT defines




WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
void messageHandler(char* topic, byte* payload, unsigned int length);
 
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
 
 
void messageHandler(char* topic, byte* payload, unsigned int length)
{
  
  Serial.println("**************");
  Serial.print("incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload,length);
  const char* message = doc["message"];
    Serial.println();
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]); // Pring payload content
  }
    const char* ledControl = doc["led_Control"];
    char led = ledControl[0];
    Serial.print("Command: ");
    Serial.println(led);
  
  if (led == 49) // 49 is the ASCI value of 1
  {
    digitalWrite(lamp1, HIGH);
    Serial.println("Lamp_State changed to HIGH");
    
  }
  else if (led == 48) // 48 is the ASCI value of 0
  {
    digitalWrite(lamp1, LOW);
    Serial.println("Lamp_State changed to LOW");
   
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