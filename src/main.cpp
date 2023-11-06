// INCLUDES
#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Credentials.h>

// DEFINES

char location_id[8];

WiFiClientSecure espClient;
PubSubClient client(espClient);

// CODE
void setup_wifi()
{
  delay(10);
  WiFiManager wifiManager;
  // configure WiFiManager to include fields for Location, Room and Project IDs
  WiFiManagerParameter custom_ids("server", "mqtt server", location_id, 40);
  wifiManager.addParameter(&custom_ids);

  wifiManager.autoConnect("IntellisensorySetup");
}


void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    
    String client_id_str = "Intellisensory" + String(random(0xffff), HEX);
    char* client_id = (char*) client_id_str.c_str();
    if (client.connect(client_id))
    {
      Serial.println(" Mqtt connected");
    }
    else
    {
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void setup()
{
  Serial.begin(9600);
  setup_wifi();
  client.setServer(MQTT_BROKER, MQTT_PORT);
}


void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  delay(1000);
  client.publish("Noe1337", "Hallo Welt");
}