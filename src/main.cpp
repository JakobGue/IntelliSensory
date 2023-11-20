// Includes
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <WiFiManager.h>
#include <Credentials.h>
#include <DallasTemperature.h>
#include <OneWire.h>

//Defines

const char* topic =   "Noe1337";
String clientId = "WeMos_";
char clientID_c_str[30] = "WeMos_";

const int output = LED_BUILTIN;
char message[50];
const int sampleWindow = 50;
unsigned int sample;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

#define SENSOR_PIN A0
#define ONE_WIRE_BUS_TEMP D2
OneWire oneWire(ONE_WIRE_BUS_TEMP);
DallasTemperature temp_sensor(&oneWire);


// Code

void setup_wifi()
{
  delay(10);
  WiFiManager wifiManager;
  // configure WiFiManager to include fields for Location, Room and Project IDs
  // WiFiManagerParameter custom_ids("server", "mqtt server", location_id, 40);
  // wifiManager.addParameter(&custom_ids);

  wifiManager.autoConnect("IntellisensorySetup");
}


void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.setClientId(clientID_c_str);
  mqttClient.connect();
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
}


long long int macToInt(const uint8_t* mac){
  long long int result = 1;
  for (int i = 0; i < 6; ++i) {
    result *= mac[i];
  }
  return result;
}

void composeClientID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);                 //MAC Addr lesen
  long long int macInt = macToInt(mac); //MAC als int
  char buffer1[20];                     //MAC als C_string
  itoa(macInt,buffer1,16);
  strcat(clientID_c_str, buffer1);      //clientID erstellen
  Serial.println(clientID_c_str);
}

void setup() {
  pinMode(SENSOR_PIN,INPUT);
  Serial.begin(115200);
  composeClientID() ;

  setup_wifi();
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToMqtt();
  
}

float read_temp_heater(){
  float retval;
  temp_sensor.requestTemperatures();
  retval = temp_sensor.getTempCByIndex(0);
  return retval;
}

float read_loudness(){
  float retval;
  unsigned long startMillis= millis();                   // Start of sample window
  float peakToPeak = 0;                                  // peak-to-peak level
 
  unsigned int signalMax = 0;                            //minimum value
  unsigned int signalMin = 1024;                         //maximum value
 
                                                          // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
      sample = analogRead(SENSOR_PIN);                    //get reading from microphone
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
      //Serial.print("Lautstärke: ");
      //Serial.println(sample)
   }
 
  peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
  int db = map(peakToPeak,20,900,49.5,90);             //calibrate for deciBels
  retval = db;
  return retval;
}


//unsigned long int newTime, lastTime, period = 10000;
void loop() {
  unsigned long startMillis= millis();                   // Start of sample window
  float peakToPeak = 0;                                  // peak-to-peak level
 
  unsigned int signalMax = 0;                            //minimum value
  unsigned int signalMin = 1024;                         //maximum value
 
                                                          // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
      sample = analogRead(SENSOR_PIN);                    //get reading from microphone
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
 
  peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
  int db = map(peakToPeak,20,900,49.5,90); 
  float temperature_heater = read_temp_heater();
  String loudness_str = String(db)+ " db";
  String temperature_heater_str = String(temperature_heater) + " °C";
  mqttClient.publish(topic, 0, false, temperature_heater_str.c_str());
  mqttClient.publish(topic, 0, false, loudness_str.c_str());
  Serial.println(loudness_str);
  Serial.println(temperature_heater_str);
  delay(5000);
}

