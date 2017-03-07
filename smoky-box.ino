

#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PID_v1.h>
#include <EEPROM.h>

#include "secrets.h"
/*
 * Secrets are: 
 *   #define WIFIPASSWORD "secret"
 *   #define MQTTPASSWORD "secret"
 *   #define UPDATEOTAPASSWORD "secret"
 */


#define SMOKY_ID "1"
#define PRODUCTION 1 // 1 to go Live, 0 to test

unsigned long looptime = 0;

// WiFi
const char* wifiSsid = "Xtreme";
const char* wifiPassword = WIFIPASSWORD;
WiFiClient  wifiClient;

// MQTT
const char* mqttServer = "10.0.0.15";
const char* mqttUser = "ottomq";
const char* mqttPassword = MQTTPASSWORD;
const long mqttPort = 1883;
const long mqttInterval = 10000;
unsigned long mqttPreviousSentMillis = 0;
PubSubClient mqttClient(wifiClient);

// MQTT Topics
#define MQTT_TOPIC_OUT "smoky/" SMOKY_ID
#define MQTT_TOPIC_IN "smoky/" SMOKY_ID "/in"
const char* mqttTopicInsideTemperature = MQTT_TOPIC_OUT "/inside/temperature";
const char* mqttTopicInsideHumidity = MQTT_TOPIC_OUT "/inside/humidity";
const char* mqttTopicOutsideTemperature = MQTT_TOPIC_OUT "/outside/temperature";
const char* mqttTopicOutsideHumidity = MQTT_TOPIC_OUT "/outside/humidity";
const char* mqttTopicSmoke = MQTT_TOPIC_OUT "/inside/smoke";
const char* mqttTopicHeaterPercent = MQTT_TOPIC_OUT "/inside/heaterpercent";
const char* mqttTopicHeaterOutput = MQTT_TOPIC_OUT "/inside/heateroutput";
const char* mqttTopicOutSettingsTemp = MQTT_TOPIC_OUT "/settings/temp";
const char* mqttTopicOutSettingsKp = MQTT_TOPIC_OUT "/settings/kp";
const char* mqttTopicOutSettingsKi = MQTT_TOPIC_OUT "/settings/ki";
const char* mqttTopicOutSettingsKd = MQTT_TOPIC_OUT "/settings/kd";
const char* mqttTopicOutSettingsFanVent = MQTT_TOPIC_OUT "/settings/fan_vent";
const char* mqttTopicOutSettingsFanCirc = MQTT_TOPIC_OUT "/settings/fan_circ";
const char* mqttTopicOutSettingsHeaterOutput = MQTT_TOPIC_OUT "/settings/heateroutput";
const char* mqttTopicOutConnecting = MQTT_TOPIC_OUT "/connecting";
const char* mqttTopicDhtReadFailures = MQTT_TOPIC_OUT "/dhtreadfailures";
const char* mqttTopicOutMsg = MQTT_TOPIC_OUT "/msg";
const char* mqttTopicFanVentSetting = MQTT_TOPIC_OUT "/fan_vent/setting";
const char* mqttTopicFanCircSetting = MQTT_TOPIC_OUT "/fan_circ/setting";
const char* mqttTopicFanVentSpeed = MQTT_TOPIC_OUT "/fan_vent/speedRPM";
const char* mqttTopicFanCircSpeed = MQTT_TOPIC_OUT "/fan_circ/speedRPM";
const char* mqttTopicInAck = MQTT_TOPIC_IN "/ack";
const char* mqttTopicInSetTemp = MQTT_TOPIC_IN "/temp";
const char* mqttTopicInSetKp = MQTT_TOPIC_IN "/kp";
const char* mqttTopicInSetKi = MQTT_TOPIC_IN "/ki";
const char* mqttTopicInSetKd = MQTT_TOPIC_IN "/kd";
const char* mqttTopicInSetFanVent = MQTT_TOPIC_IN "/fan_vent";
const char* mqttTopicInSetFanCirc = MQTT_TOPIC_IN "/fan_circ";
const char* mqttTopicInReadSettings = MQTT_TOPIC_IN "/read";


// MQTT text messages
const char* mqttMsgInsideTempNan = "Error reading inside temperature";

// MQTT Status
#define MQTT_STATUS_UNKNOWN 0
#define MQTT_STATUS_SENDING 1
#define MQTT_STATUS_ACKNOWLEDGED 2
#define MQTT_STATUS_FAILED 3
#define GREENLED 13
const long ledInterval = 250;
unsigned int mqttStatus = MQTT_STATUS_UNKNOWN;
bool greenLed = false;
unsigned long ledPreviousMillis = 0;

// Smoke
#define SMOKEPIN A0


// Temperature and humidity
#define DHTINSIDE 2 // D4
#define DHTOUTSIDE 4 // D2
#define DHTTYPE DHT22
DHT dhtInside(DHTINSIDE, DHTTYPE);
DHT dhtOutside(DHTOUTSIDE, DHTTYPE);
double temperatureInside =  5.0;
float humidityInside = 0.0;
float temperatureOutside = 0.0;
float humidityOutside = 0.0;
unsigned long dhtReadFailures = 0;
unsigned long dhtReadInterval = 2000; 
unsigned long dhtPreviousReadMillis = 0;

// EEPROM Settings
#define SETTINGS_VERSION 100
#define DEFAULT_KP 5.0
#define DEFAULT_KI 0.05
#define DEFAULT_KD 0.5
#define DEFAULT_TEMP 5.0
#define DEFAULT_FAN 10
struct SmokySettings {
  int version; // Figure ot if data has been saved before
  float Kp;
  float Ki;
  float Kd;
  double temp;
  int fanVent;
  int fanCirc;
};
SmokySettings settings;


// Heater control
#define HEATERPIN 12 // D6
unsigned long pidWindowSize = 5000;
unsigned long pidLoopTime = 1000;
unsigned long maxHeaterOutput = 100;
bool heaterOn = false;
unsigned long heaterPreviousCalcMillis = 0;
unsigned long heaterPreviousPidWindow = 0;
double heaterOutput =  10.0;
PID heaterPID(&temperatureInside, &heaterOutput, &(settings.temp), settings.Kp, settings.Ki, settings.Kd, DIRECT);


// Smoke
float smoke = 0.0;

// OTA Update
// http://esp8266.github.io/Arduino/versions/2.0.0/doc/ota_updates/ota_updates.html
const char* host = "smoky-update";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = UPDATEOTAPASSWORD;
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// Fan ventilation
#define FAN_VENT_SENSOR_PIN 1 // D10 or TX
#define FAN_VENT_PWM_PIN 3 // D9 or RX
unsigned long fanVentSpeedRPM = 0; // Actual speed
volatile unsigned long fanVentCount = 0;

// Fan circulation
#define FAN_CIRC_SENSOR_PIN 13 // D7
#define FAN_CIRC_PWM_PIN 15 // D8
unsigned long fanCircSpeedRPM = 0; // Actual speed
volatile unsigned long fanCircCount = 0;

// Fan common
unsigned long previousFanMillis = 0;
unsigned long fanInterval = 500;


void setup() {

  // WiFi
  delay(10);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Temperature and humidity
  dhtInside.begin();
  dhtOutside.begin();
  dhtPreviousReadMillis = 0;
  loopReadTempHumid(); // Initial reading to get the temp controller a better start.

  // MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  randomSeed(micros());
  pinMode(GREENLED, OUTPUT);

  delay(100);

  // Heater
  pinMode(HEATERPIN, OUTPUT);
  heaterPID.SetOutputLimits(0, maxHeaterOutput);
  heaterPID.SetMode(AUTOMATIC);
  heaterPreviousPidWindow = millis();


  EEPROM.begin(256);
  readSettingsFromEEPROM();


  // OTA Update
  MDNS.begin("smoky");
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);

  // LED
  ledPreviousMillis = 0;

  // Fan ventilation
  pinMode(FAN_VENT_PWM_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_VENT_SENSOR_PIN), countFanVent, RISING);
  pinMode(FAN_VENT_SENSOR_PIN, INPUT_PULLUP);

  // Fan circulation
  pinMode(FAN_CIRC_PWM_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_CIRC_SENSOR_PIN), countFanCirc, RISING);
  pinMode(FAN_CIRC_SENSOR_PIN, INPUT_PULLUP);

  writeMqttSettings();

}

void readSettingsFromEEPROM(){
  int eeAddress = 0;
  mqttSend(mqttTopicOutMsg, String("Reading EEPROM").c_str());
  EEPROM.get(eeAddress, settings);
  
  if(settings.version != SETTINGS_VERSION){
    mqttSend(mqttTopicOutMsg, String("EEPROM - Not found version").c_str());
    settings.version = SETTINGS_VERSION;
    settings.Kp = DEFAULT_KP;
    settings.Ki = DEFAULT_KI;
    settings.Kd = DEFAULT_KD;
    settings.temp = DEFAULT_TEMP;
    settings.fanVent = DEFAULT_FAN;
    settings.fanCirc = DEFAULT_FAN;
    EEPROM.put(0, settings);
  }
}

void writeSettingsToEEPROM(){
  int eeAddress = 0;
  mqttSend(mqttTopicOutMsg, String("Writing EEPROM").c_str());
  EEPROM.put(eeAddress, settings);
  EEPROM.commit();
}

void loop() {
  looptime = millis();
  loopBlinkLed();
  loopReadTempHumid();
  loopBlinkLed();
  loopControlHeater();
  loopBlinkLed();
  smoke = analogRead(SMOKEPIN);
  loopBlinkLed();
  loopWriteMqtt();
  loopBlinkLed();
  mqttClient.loop();
  loopBlinkLed();
  loopFan();

  // OTA Update
  httpServer.handleClient();
}

void loopReadTempHumid(){
  unsigned long currentMillis = millis();
  if (currentMillis - dhtPreviousReadMillis > dhtReadInterval) {
    double prevTi = temperatureInside;
    temperatureInside =  dhtInside.readTemperature();
    if (isnan(temperatureInside)) {
      temperatureInside = prevTi;
      dhtReadFailures++;
      if(PRODUCTION) mqttSend(mqttTopicOutMsg, String("DHT22 Read error").c_str());
    }
    humidityInside = dhtInside.readHumidity();
    temperatureOutside = dhtOutside.readTemperature();
    humidityOutside = dhtOutside.readHumidity();
    dhtPreviousReadMillis = currentMillis;
  }
}

void loopControlHeater(){

  unsigned long currentMillis = looptime;

  if (currentMillis >= (heaterPreviousCalcMillis + pidLoopTime)) {
  
    heaterPID.Compute();
    
    if (currentMillis - heaterPreviousPidWindow > pidWindowSize) { //time to shift the Relay Window
      heaterPreviousPidWindow += pidWindowSize;
    }
    if (heaterOutput > maxHeaterOutput * (currentMillis - heaterPreviousPidWindow) / pidWindowSize) {
      digitalWrite(HEATERPIN, HIGH); // On
      heaterOn = true;
    } else {
      digitalWrite(HEATERPIN, LOW); // Off
      heaterOn = false;
    }
    heaterPreviousCalcMillis = currentMillis;  
  }
}

void loopWriteMqtt(){
  unsigned long currentMillis = looptime;
  if(currentMillis - mqttPreviousSentMillis >= mqttInterval) {
    writeMqtt();
    mqttPreviousSentMillis = currentMillis;
  }  
}

void loopFan(){
  unsigned long currentMillis = millis();
  if(currentMillis - previousFanMillis >= fanInterval) {
    unsigned long countTime = currentMillis - previousFanMillis;
    noInterrupts();
    fanVentSpeedRPM = (fanVentCount * (30000 / countTime)); // 1000 ms * 60 sek / 2 counts per round
    fanVentCount = 0;
    fanCircSpeedRPM = (fanCircCount * (30000 / countTime)); // 1000 ms * 60 sek / 2 counts per round
    fanCircCount = 0;
    interrupts();
    previousFanMillis = currentMillis;
    analogWrite(FAN_VENT_PWM_PIN, map(settings.fanVent, 0, 100, 0, 1023));
    analogWrite(FAN_CIRC_PWM_PIN, map(settings.fanCirc, 0, 100, 0, 1023));
  }
}

void countFanVent(){ 
  fanVentCount++;
}

void countFanCirc(){ 
  fanCircCount++;
}

void mqttSend(const char* topic, const char* msg){
  reconnectMqtt();
  mqttClient.publish(topic, msg); 
}

void loopBlinkLed(){
  /* 
   * Control the green LED.
   * Start blinking when data is sent to MQTT
   * Constant on when ACK is received from MQTT
   * TO DO: Constant off if no ACK is received within the timeout.
   *
   * I am calling this multiple times in the loop, to get a more stable blinking frequency.
   */
  unsigned long currentMillis = millis();
  if(currentMillis - ledPreviousMillis >= ledInterval) {
    if(mqttStatus == MQTT_STATUS_ACKNOWLEDGED){
      greenLed = true;
    } else if(mqttStatus == MQTT_STATUS_SENDING){
      greenLed = not greenLed;
    } else {
      greenLed = false;
    }
    ledPreviousMillis = currentMillis;
    if(greenLed) {
      digitalWrite(GREENLED, HIGH);
    } else {
      digitalWrite(GREENLED, LOW);
    }
  }
  
}


void writeMqtt(){
  mqttStatus = MQTT_STATUS_SENDING;
  reconnectMqtt();

  mqttClient.publish(mqttTopicInsideTemperature, String(temperatureInside).c_str());
  mqttClient.publish(mqttTopicInsideHumidity, String(humidityInside).c_str());
  mqttClient.publish(mqttTopicOutsideTemperature, String(temperatureOutside).c_str());
  mqttClient.publish(mqttTopicOutsideHumidity, String(humidityOutside).c_str());
  mqttClient.publish(mqttTopicSmoke, String(smoke).c_str());
  mqttClient.publish(mqttTopicHeaterOutput, (String(heaterOutput)).c_str());
  mqttClient.publish(mqttTopicDhtReadFailures, (String(dhtReadFailures)).c_str());
  mqttClient.publish(mqttTopicFanVentSpeed, (String(fanVentSpeedRPM)).c_str());
  mqttClient.publish(mqttTopicFanCircSpeed, (String(fanCircSpeedRPM)).c_str());
}

void writeMqttSettings(){
  mqttStatus = MQTT_STATUS_SENDING;
  reconnectMqtt();

  mqttClient.publish(mqttTopicOutSettingsTemp, String(settings.temp).c_str());
  mqttClient.publish(mqttTopicOutSettingsHeaterOutput, String(heaterOutput).c_str());
  mqttClient.publish(mqttTopicOutSettingsKp, String(settings.Kp).c_str());
  mqttClient.publish(mqttTopicOutSettingsKi, String(settings.Ki).c_str());
  mqttClient.publish(mqttTopicOutSettingsKd, String(settings.Kd).c_str());
  mqttClient.publish(mqttTopicOutSettingsFanVent, String(settings.fanVent).c_str());
  mqttClient.publish(mqttTopicOutSettingsFanCirc, String(settings.fanCirc).c_str());
}

void reconnectMqtt() {
  if (!mqttClient.connected()) {
    // Create a random client ID
    String clientId = "smoky-" SMOKY_ID;
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      // Once connected, publish an announcement...
      if(mqttStatus == MQTT_STATUS_UNKNOWN) {
        mqttClient.publish(mqttTopicOutConnecting, "Smoky starting");
      } else {
        mqttClient.publish(mqttTopicOutConnecting, "Smoky reconnecting");
      }

      mqttClient.subscribe(MQTT_TOPIC_IN "/#", 1);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if(strcmp(topic, mqttTopicInAck) == 0) {
    // Set status to turn on green led if ack is received
    mqttStatus = MQTT_STATUS_ACKNOWLEDGED;
  } else if(strcmp(topic, mqttTopicInSetTemp) == 0) {
    // Set temp
    settings.temp = payloadToFloat(payload, length, settings.temp);
    writeSettingsToEEPROM();
    mqttSend(mqttTopicOutSettingsTemp, String(settings.temp).c_str());
  } else if(strcmp(topic, mqttTopicInSetKp) == 0) {
    // Set Kp
    settings.Kp = payloadToFloat(payload, length, settings.Kp);
    heaterPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    writeSettingsToEEPROM();
    mqttSend(mqttTopicOutSettingsKp, String(settings.Kp).c_str());
  } else if(strcmp(topic, mqttTopicInSetKi) == 0) {
    // Set Ki
    settings.Ki = payloadToFloat(payload, length, settings.Ki);
    heaterPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    writeSettingsToEEPROM();
    mqttSend(mqttTopicOutSettingsKi, String(settings.Ki).c_str());
  } else if(strcmp(topic, mqttTopicInSetKd) == 0) {
    // Set Kd
    settings.Kd = payloadToFloat(payload, length, settings.Kd);
    heaterPID.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    writeSettingsToEEPROM();
    mqttSend(mqttTopicOutSettingsKd, String(settings.Kd).c_str());
  } else if(strcmp(topic, mqttTopicInSetFanVent) == 0) {
    // Set ventilation fan speed
    int setting = payloadToInt(payload, length, settings.fanVent);
    if (setting < 0) setting = 0;
    if (setting > 100) setting = 100;
    settings.fanVent = setting;
    writeSettingsToEEPROM();
    mqttSend(mqttTopicOutSettingsFanVent, String(settings.fanVent).c_str());
  } else if(strcmp(topic, mqttTopicInSetFanCirc) == 0) {
    // Set circulation fan speed
    int setting = payloadToInt(payload, length, settings.fanCirc);
    if (setting < 0) setting = 0;
    if (setting > 100) setting = 100;
    settings.fanCirc = setting;
    writeSettingsToEEPROM();
    mqttSend(mqttTopicOutSettingsFanCirc, String(settings.fanCirc).c_str());
  } else if(strcmp(topic, mqttTopicInReadSettings) == 0) {
    // Read settings
    writeMqttSettings();
  }
}

double payloadToFloat(byte* payload, unsigned int length, double defaultValue){
  payload[length] = '\0';
  String s = String((char*)payload);
  double f = (double) s.toFloat();
  if (f == 0.0 && payload[0] != '0') return defaultValue;
  else return f;
}

int payloadToInt(byte* payload, unsigned int length, int defaultValue){
  payload[length] = '\0';
  String s = String((char*)payload);
  int f = s.toInt();
  if (f == 0 && payload[0] != '0') return defaultValue;
  else return f;
}

