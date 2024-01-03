#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Credentials.h>
#include <DateTime.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "helperfunction.h"
#include "WiFiManager.h"

char message[50];
const int sampleWindow = 50;
unsigned int sample;

#define SENSOR_PIN A0
#define DEBUG_PIN D4
#define ONE_WIRE_BUS_TEMP D3
OneWire oneWire(ONE_WIRE_BUS_TEMP);
DallasTemperature temp_sensor(&oneWire);
SensirionI2CScd4x scd4x;


int i = 0;

const char* ssid     = SSID ;
const char* password = WIFI_PWD;
const char* server = MQTT_HOST;

const String location = LOCATION;
const String room = ROOM;
const float sound_threshold = 1.2;
const String clientId = location + "_IntelliSensory_" + room;

float sound_cycle[120];

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}
WiFiClientSecure espClient;
PubSubClient client(server, 15, callback, espClient);

void setupDateTime(){
  DateTime.setServer("time1.hs-augsburg.de", "time2.hs-augsburg.de", "time3.hs-augsburg.de");
  DateTime.setTimeZone("UTC-1");
  DateTime.begin();
  Serial.println(DateTime.toISOString());
}

void reconnect() 
{
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "IntelliSensory_" + location + "_" + room + "_" + MODE;

    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" Mqtt connected");
    }else {
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println('\n');
  pinMode(DEBUG_PIN, INPUT);
  
  WiFi.begin(ssid, password);  
  espClient.setInsecure();           // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());  
  if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("connected");
  }

  Wire.begin();
  setupDateTime();
  Serial.println(DateTime.toISOString());

  if (MODE == "DESK") {
    scd4x.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();

    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");
  }
}

float read_temp_heater(){
  float retval;
  retval = 1.0;
  temp_sensor.requestTemperatures();
  retval = temp_sensor.getTempCByIndex(0);
  return retval;
}

float read_loudness(){
  unsigned long startMillis= millis();                   // Start of sample window
  float peakToPeak = 0;                                  // peak-to-peak level
 
  unsigned int signalMax = 0;                            //minimum value
  unsigned int signalMin = 1024;                         //maximum value
 
                                                          // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
      sample = analogRead(SENSOR_PIN);            //get reading from microphone
      if (sample < 1024)                                  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;                           // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;                           // save just the min levels
         }
      }
   }
 
  peakToPeak = signalMax - signalMin;  
  float signal = ((peakToPeak * 5.0) / 1024);                 // max - min = peak-peak amplitude
  return signal;
}


void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  Serial.println(i);
  if (i == 120) {

    String result = "";
    if (MODE == "DESK") {
      uint16 co2;
      float temperature_room, humidity;
      scd4x.readMeasurement(co2, temperature_room, humidity);
      float currentMax = 0;
      for (int i = 0; i < (sizeof(sound_cycle) / sizeof(sound_cycle[0])); i++) {
        if (sound_cycle[i] > currentMax) {
          currentMax = sound_cycle[i];
        }
      }
      Serial.println(currentMax);
      bool sound = (currentMax >= sound_threshold);
      String formattedDate = DateTime.toISOString();
    
      const SensorData sensorData[] = {
            {5, sound, formattedDate},
            {3, temperature_room, formattedDate},
            {1, co2, formattedDate},
            {4, humidity, formattedDate}
        };
      result = createJson(false, sensorData, sizeof(sensorData) / sizeof(sensorData[0]));
      Serial.println(result);
      Serial.println(sizeof(result.c_str()));
    }else {
      float temperature_heater = read_temp_heater();
      String temperature_heater_str = String(temperature_heater) + " °C";
      String formattedDate = DateTime.toISOString();
    
      const SensorData sensorData[] = {
            {1, temperature_heater, formattedDate}
        };
      
      result = createJson(false, sensorData, sizeof(sensorData) / sizeof(sensorData[0]));
      Serial.println(result.c_str());
    }

    String topic = "se/i2projekt/" + location + "/" + room + "/intellisensory";
    Serial.println(topic);
    client.publish(topic.c_str(), result.c_str());

    i = 0;
    }

  // delaying whole 60s at once results in connection loss, therefore delay 500ms 120 times
  delay(450);
  sound_cycle[i] = read_loudness();
  i++;
}
