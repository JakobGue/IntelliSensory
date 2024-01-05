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
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>

char message[50];
const int sampleWindow = 50;
unsigned int sample;

char output_room[20] = "";
char output_location[20] = "";
char output_mode[7] = "";
bool shouldSaveConfig = false;

#define SENSOR_PIN A0
#define DEBUG_PIN D4
#define ONE_WIRE_BUS_TEMP D3
OneWire oneWire(ONE_WIRE_BUS_TEMP);
DallasTemperature temp_sensor(&oneWire);
SensirionI2CScd4x scd4x;

String location;
String room;
String mode;

String clientId;
int i = 0;
char *server = MQTT_HOST;

const float sound_threshold = 1.2;
float sound_cycle[120];

void callback(char *topic, byte *payload, unsigned int length)
{
  // handle message arrived
}
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
WiFiClientSecure espClient;
PubSubClient client(server, 15, callback, espClient);

void setupDateTime()
{
  DateTime.setServer("time1.hs-augsburg.de", "time2.hs-augsburg.de", "time3.hs-augsburg.de");
  DateTime.setTimeZone("UTC-1");
  DateTime.begin();
  Serial.println(DateTime.toISOString());
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    clientId = "IntelliSensory_" + location + "_" + room + "_" + mode;
    clientId.toLowerCase();
    String username;
    String password;
    if (DEBUG_PIN == HIGH)
    {
      username = DEBUG_USER;
      password = DEBUG_PASSWORD;
    }
    else
    {
      username = MQTT_USER;
      password = MQTT_PASSWORD;
    }
    if (client.connect(clientId.c_str(), username.c_str(), password.c_str()))
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
  if (DEBUG_PIN == HIGH)
  {
    Serial.println("Debug mode");
    client.setServer(DEBUG_HOST, DEBUG_PORT);
  }
  Serial.begin(115200);
  delay(10);
  Serial.println('\n');
  pinMode(DEBUG_PIN, INPUT);

  if (SPIFFS.begin())
  {
    if (DEBUG_PIN == HIGH)
    {
      SPIFFS.format();
    }
    SPIFFS.format();
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      // file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        StaticJsonDocument<256> json;
        DeserializationError jsonError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);

        if (!jsonError)
        {
          Serial.println("\nparsed json");
          strcpy(output_room, json["output_room"]);
          strcpy(output_location, json["output_location"]);
          strcpy(output_mode, json["output_mode"]);
          room = String(output_room);
          location = String(output_location);
          mode = String(output_mode);
          room.trim();
          room.toLowerCase();
          location.trim();
          location.toLowerCase();
          mode.trim();
          mode.toUpperCase();
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }

  espClient.setInsecure();
  
  WiFiManagerParameter custom_output_loc("loc", "Location", output_location, 20);
  WiFiManagerParameter custom_output_room("room", "Room", output_room, 20);
  WiFiManagerParameter custom_output_mode("mode", "Mode", output_mode, 7);

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_output_room);
  wifiManager.addParameter(&custom_output_loc);
  wifiManager.addParameter(&custom_output_mode);

  wifiManager.autoConnect("Intellisensory_Setup");

  Serial.println("Connected.");

  strcpy(output_room, custom_output_room.getValue());
  strcpy(output_location, custom_output_loc.getValue());
  strcpy(output_mode, custom_output_mode.getValue());
  room = String(output_room);
  location = String(output_location);
  mode = String(output_mode);
  room.trim();
  room.toLowerCase();
  location.trim();
  location.toLowerCase();
  mode.trim();
  mode.toUpperCase();

  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    StaticJsonDocument<256> json;
    json["output_room"] = output_room;
    json["output_location"] = output_location;
    json["output_mode"] = output_mode;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }

    serializeJsonPretty(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
  }

  int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(++i);
    Serial.print(' ');
  }

  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());
  clientId = "IntelliSensory_" + location + "_" + room + "_" + mode;
  clientId.toLowerCase();
  if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
  {
    Serial.println("connected");
  }

  Wire.begin();
  setupDateTime();
  Serial.println(DateTime.toISOString());

  if (mode == "DESK")
  {
    scd4x.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = scd4x.stopPeriodicMeasurement();
    if (error)
    {
      Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }

    error = scd4x.startPeriodicMeasurement();

    if (error)
    {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");
  }
}

float read_temp_heater()
{
  float retval;
  retval = 1.0;
  temp_sensor.requestTemperatures();
  retval = temp_sensor.getTempCByIndex(0);
  return retval;
}

float read_loudness()
{
  unsigned long startMillis = millis(); 
  float peakToPeak = 0;                 

  unsigned int signalMax = 0;    
  unsigned int signalMin = 1024; 

  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(SENSOR_PIN); 
    if (sample < 1024)               
    {
      if (sample > signalMax)
      {
        signalMax = sample; 
      }
      else if (sample < signalMin)
      {
        signalMin = sample; 
      }
    }
  }

  peakToPeak = signalMax - signalMin;
  float signal = ((peakToPeak * 5.0) / 1024); 
  return signal;
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  Serial.println(i);
  if (i == 120)
  {

    String result = "";
    if (mode == "DESK")
    {
      uint16 co2;
      float temperature_room, humidity;
      scd4x.readMeasurement(co2, temperature_room, humidity);
      float currentMax = 0;
      for (int i = 0; i < (sizeof(sound_cycle) / sizeof(sound_cycle[0])); i++)
      {
        if (sound_cycle[i] > currentMax)
        {
          currentMax = sound_cycle[i];
        }
      }
      Serial.println(currentMax);
      bool sound = (currentMax >= sound_threshold);
      String formattedDate = DateTime.toISOString();

      const SensorData sensorData[] =
          {
              {5, sound, formattedDate},
              {3, temperature_room, formattedDate},
              {1, co2, formattedDate},
              {4, humidity, formattedDate}};
      result = createJson(false, sensorData, sizeof(sensorData) / sizeof(sensorData[0]));
      Serial.println(result);
      Serial.println(sizeof(result.c_str()));
    }
    else
    {
      float temperature_heater = read_temp_heater();
      String formattedDate = DateTime.toISOString();

      const SensorData sensorData[] =
          {
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
  
  if (mode == "DESK") 
  {
    delay(450);
    sound_cycle[i] = read_loudness();
  } else {
    delay(500);
  }
  i++;
}
