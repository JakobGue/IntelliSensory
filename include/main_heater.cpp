// Includes
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <WiFiManager.h>
#include <Credentials.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DateTime.h>
#include <iostream>
#include <vector>
#include <tuple>
#include "helperfunction.h"



//Defines

const char* topic =   "Noe1337";
String clientId = "WeMos_";
char clientID_c_str[30] = "WeMos_";


AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;



const int output = LED_BUILTIN;
char message[50];
const int sampleWindow = 50;
unsigned int sample;

#define SENSOR_PIN A0
#define DEBUG_PIN D4
#define ONE_WIRE_BUS_TEMP D3
OneWire oneWire(ONE_WIRE_BUS_TEMP);
DallasTemperature temp_sensor(&oneWire);


// Code
void setupDateTime(){
  DateTime.setServer("de.pool.ntp.org");
  DateTime.setTimeZone("UTC");
  DateTime.begin();
}


void setup_wifi()
{
  delay(10);
  Serial.println("Connecting to WiFi");
  WiFiManager wifiManager;
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
  if (sessionPresent == 0){
    Serial.println("Keine Internetverbindung");
  }
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
  Serial.begin(115200);
  pinMode(DEBUG_PIN, INPUT);
  composeClientID() ;
  setup_wifi();
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Connecting to Wifi...");
  }
  //Serial.println("ERREICHT");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  mqttClient.onMessage(onMqttMessage);
  if (digitalRead(DEBUG_PIN) == HIGH){
    mqttClient.setServer(DEBUG_HOST, DEBUG_PORT);
  }
  else{
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  }

  connectToMqtt();

  setupDateTime();
  
  pinMode(SENSOR_PIN, INPUT);
  
  Wire.begin();
}

float read_temp_heater(){
  float retval;
  temp_sensor.requestTemperatures();
  retval = temp_sensor.getTempCByIndex(0);
  return retval;
}

//unsigned long int newTime, lastTime, period = 10000;
void loop() {                          // peak-to-peak level

  float temperature_heater = read_temp_heater();

  String temperature_heater_str = String(temperature_heater) + " °C";



  String formattedDate = DateTime.toISOString();
  
  const SensorData sensorData[] = {
        {1, temperature_heater, formattedDate}
    };
// Output from helperfunction - state of debugflag and sensordata
  String result = createJson(false, sensorData, sizeof(sensorData) / sizeof(sensorData[0]));
  
  mqttClient.publish(topic, 0, false, result.c_str());
  Serial.print("Temperatur an Heizung: ");
  Serial.print("\t");
  Serial.println(temperature_heater_str);
  Serial.print("Datum:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");

  Serial.println(DateTime.toISOString());
  
  Serial.println("************************************************");

  delay(5000);
}

