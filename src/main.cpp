// Includes

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

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

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


unsigned long int newTime, lastTime, period = 10000;
void loop() {
  float temperature_heater = read_temp_heater();
  String temperature_heater_str = String(temperature_heater) + " °C";
  mqttClient.publish(topic, 0, false, temperature_heater_str.c_str());
  delay(5000);
}

