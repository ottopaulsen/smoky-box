

#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PID_v1.h>
#include <FanController.h>

#include "secrets.h"
/*
 * Secrets are: 
 *   #define WIFIPASSWORD "secret"
 *   #define MQTTPASSWORD "secret"
 *   #define UPDATEOTAPASSWORD "secret"
 */

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
#define MQTT_TOPIC_OUT "smoky/1"
#define MQTT_TOPIC_IN "smoky/1/in"
const char* mqttTopicInsideTemperature = MQTT_TOPIC_OUT "/inside/temperature";
const char* mqttTopicInsideHumidity = MQTT_TOPIC_OUT "/inside/humidity";
const char* mqttTopicOutsideTemperature = MQTT_TOPIC_OUT "/outside/temperature";
const char* mqttTopicOutsideHumidity = MQTT_TOPIC_OUT "/outside/humidity";
const char* mqttTopicSmoke = MQTT_TOPIC_OUT "/inside/smoke";
const char* mqttTopicHeaterPercent = MQTT_TOPIC_OUT "/inside/heaterpercent";
const char* mqttTopicHeaterOutput = MQTT_TOPIC_OUT "/inside/heateroutput";
const char* mqttTopicOutSettings = MQTT_TOPIC_OUT "/settings";
const char* mqttTopicOutConnecting = MQTT_TOPIC_OUT "/connecting";
const char* mqttTopicDhtReadFailures = MQTT_TOPIC_OUT "/dhtreadfailures";
const char* mqttTopicOutMsg = MQTT_TOPIC_OUT "/msg";
const char* mqttTopicFanSetting = MQTT_TOPIC_OUT "/fan/setting";
const char* mqttTopicFanSpeed = MQTT_TOPIC_OUT "/fan/speedRPM";
const char* mqttTopicInAck = MQTT_TOPIC_IN "/ack";
const char* mqttTopicInSetTemp = MQTT_TOPIC_IN "/temp";
const char* mqttTopicInSetKp = MQTT_TOPIC_IN "/kp";
const char* mqttTopicInSetKi = MQTT_TOPIC_IN "/ki";
const char* mqttTopicInSetKd = MQTT_TOPIC_IN "/kd";
const char* mqttTopicInSetFan = MQTT_TOPIC_IN "/fan";
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
bool readSettings = true;

// Smoke
#define SMOKEPIN A0

// Serial
const long serialWriteInterval = 1000;
unsigned long serialPreviousWriteMillis = 0;

// Temperature and humidity
#define DHTINSIDE 2
#define DHTOUTSIDE 4
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

// Heater control
#define HEATERPIN 12
//#define INITIALTEMPSETTING 5.0
int pidWindowSize = 5000;
unsigned long maxHeaterOutput = 100;
float heaterOnPercent = 0.0;
bool heaterOn = false;
unsigned long heaterPreviousCalcMillis = 0;
unsigned long heaterPreviousPidWindow = 0;
double temperatureSetting =  5.0;
float Kp =  5.0, Ki =  0.05, Kd =  0.5;
double heaterOutput =  10.0;
PID heaterPID(&temperatureInside, &heaterOutput, &temperatureSetting, Kp, Ki, Kd, DIRECT);

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

// Fan
#define FAN_SENSOR_PIN 1
#define FAN_SENSOR_THRESHOLD 1000
#define FAN_PWM_PIN 3
FanController fan(FAN_SENSOR_PIN, FAN_SENSOR_THRESHOLD, FAN_PWM_PIN);
int fanSetting = 0; // 0-100
int fanActualSetting = 0; // 0-100
int fanSpeedRPM = 0; // Actual speed


void setup() {

  // Serial
  Serial.begin(115200);

  // WiFi
  delay(10);
  Serial.print("Connecting wifi");
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Temperature and humidity
  dhtInside.begin();
  dhtOutside.begin();
  dhtPreviousReadMillis = 0;
  loopReadTempHumid(); // Initial reading to get the temp controller a better start.

  // Heater
  pinMode(HEATERPIN, OUTPUT);
  heaterPID.SetOutputLimits(0, maxHeaterOutput);
  heaterPID.SetMode(AUTOMATIC);
  heaterPreviousPidWindow = millis();


  // MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  randomSeed(micros());
  pinMode(GREENLED, OUTPUT);

  // OTA Update
  MDNS.begin("smoky");
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);

  // LED
  ledPreviousMillis = 0;

  // Fan
  fan.begin();  

}

void loop() {
  looptime = millis();
  /* Currently there are many calls to the loopBlinkLed() method, which is controlling the 
     blink frequency of the green LED, that is blinking while waiting for ack from Node-RED.
     The reason is that it was blinking wit a variable frequency, probably because some
     of these routines takes too long time. Not a big deal, so it could be called just once.
  */
  loopBlinkLed();
  loopReadTempHumid();
  loopBlinkLed();
  loopControlHeater();
  loopBlinkLed();
  smoke = analogRead(SMOKEPIN);
  loopBlinkLed();
  loopWriteSerial();  
  loopBlinkLed();
  loopWriteMqtt();
  loopBlinkLed();
  mqttClient.loop();
  loopBlinkLed();
  loopFan();

  // OTA Update
  httpServer.handleClient();
}

// Functions called in the loop (prefixed with loop)

void loopReadTempHumid(){
  unsigned long currentMillis = millis();
  if (currentMillis - dhtPreviousReadMillis > dhtReadInterval) {
    double prevTi = temperatureInside;
    temperatureInside =  dhtInside.readTemperature();
    if (isnan(temperatureInside)) {
      temperatureInside = prevTi;
      dhtReadFailures++;
      //Serial.println("DHT22 Read error");
      mqttClient.publish(mqttTopicOutMsg, String("DHT22 Read error").c_str());
    }
    humidityInside = dhtInside.readHumidity();
    temperatureOutside = dhtOutside.readTemperature();
    humidityOutside = dhtOutside.readHumidity();
    dhtPreviousReadMillis = currentMillis;
  }
}

void loopControlHeater(){

  unsigned long currentMillis = looptime;

  if (heaterOn) {
    heaterOnPercent += (currentMillis - heaterPreviousCalcMillis) / (float) mqttInterval;
  }
  
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

void loopWriteSerial(){
  unsigned long currentMillis = millis();
  if(currentMillis - serialPreviousWriteMillis >= serialWriteInterval) {
    writeSerial();
    serialPreviousWriteMillis = currentMillis;
  }  
}

void loopWriteMqtt(){
  //reconnectMqtt();
  unsigned long currentMillis = looptime;
  if(currentMillis - mqttPreviousSentMillis >= mqttInterval) {
    writeMqtt();
    mqttPreviousSentMillis = currentMillis;
  }  
}

void loopFan(){
    fanSpeedRPM = fan.getSpeed(); // Send the command to get RPM  
    fanActualSetting = fan.getDutyCycle();
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
      Serial.println("Unknown MQTT status");
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


void writeSerial(){

  return; // Remove when needing this

  Serial.print("Fan setting: ");
  Serial.print(fanSetting);
  Serial.print("   Actual setting: ");
  Serial.print(fanActualSetting);
  Serial.print("   RPM: ");
  Serial.print(fanSpeedRPM);
  Serial.println("");

  Serial.print("Smoke level: ");
  Serial.print(smoke, 3);
  
  Serial.print("    Temperature inside: ");
  Serial.print(temperatureInside, 1);
  Serial.print(" C");
  
  Serial.print("    Temperature outside: ");
  Serial.print(temperatureOutside, 1);
  Serial.print(" C");
  
  Serial.print("    Humidity inside: ");
  Serial.print(humidityInside, 1);
  Serial.print("");

  Serial.print("    Humidity outside: ");
  Serial.print(humidityOutside, 1);
  Serial.print("");

  Serial.print("    heaterOutput: ");
  Serial.print(heaterOutput, 1);
  Serial.print("");
  
  Serial.print("    Heat %: ");
  Serial.print(heaterOnPercent, 1);
  Serial.print("");

  Serial.println("");
}

void writeMqtt(){
  mqttStatus = MQTT_STATUS_SENDING;
  reconnectMqtt();
  Serial.println("Sending MQTT data");

  mqttClient.publish(mqttTopicInsideTemperature, String(temperatureInside).c_str());
  mqttClient.publish(mqttTopicInsideHumidity, String(humidityInside).c_str());
  mqttClient.publish(mqttTopicOutsideTemperature, String(temperatureOutside).c_str());
  mqttClient.publish(mqttTopicOutsideHumidity, String(humidityOutside).c_str());
  mqttClient.publish(mqttTopicSmoke, String(smoke).c_str());
  mqttClient.publish(mqttTopicHeaterPercent, String(heaterOnPercent * 100.0).c_str());
  mqttClient.publish(mqttTopicHeaterOutput, (String(heaterOutput)).c_str());
  mqttClient.publish(mqttTopicDhtReadFailures, (String(dhtReadFailures)).c_str());
  mqttClient.publish(mqttTopicFanSpeed, (String(fanSpeedRPM)).c_str());
  mqttClient.publish(mqttTopicFanSetting, (String(fanActualSetting)).c_str());
  heaterOnPercent = 0.0;


  if (readSettings){
    mqttClient.publish(mqttTopicOutSettings, (String("temperatureSetting = ") + String(temperatureSetting)).c_str());
    mqttClient.publish(mqttTopicOutSettings, (String("heaterOutput = ") + String(heaterOutput)).c_str());
    mqttClient.publish(mqttTopicOutSettings, (String("Kp = ") + String(Kp)).c_str());
    mqttClient.publish(mqttTopicOutSettings, (String("Ki = ") + String(Ki)).c_str());
    mqttClient.publish(mqttTopicOutSettings, (String("Kd = ") + String(Kd)).c_str());
    mqttClient.publish(mqttTopicOutSettings, (String("Fan = ") + String(fanSetting)).c_str());
    readSettings = false;
  }

}

void reconnectMqtt() {
  /* 
   * This code is copied from an example.
   * Not sure it is the optimal usage of the client id.
   */

  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "smoky-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      if(mqttStatus == MQTT_STATUS_UNKNOWN) {
        mqttClient.publish(mqttTopicOutConnecting, "Smoky starting");
      } else {
        mqttClient.publish(mqttTopicOutConnecting, "Smoky reconnecting");
      }

      // ... and resubscribe
      if(mqttClient.subscribe(MQTT_TOPIC_IN "/#", 1)){
        Serial.println("Subscribe ok");
      } else {
        Serial.println("Subscribe failed");
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again next time");
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  /*
   * Callback for subscribed messages
   */
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");

  // Set status to turn on green led if ack is received
  if(strcmp(topic, mqttTopicInAck) == 0) {
    mqttStatus = MQTT_STATUS_ACKNOWLEDGED;
  } else if(strcmp(topic, mqttTopicInSetTemp) == 0) {
    temperatureSetting = payloadToFloat(payload, length, temperatureSetting);
  } else if(strcmp(topic, mqttTopicInSetKp) == 0) {
    Kp = payloadToFloat(payload, length, Kp);
    heaterPID.SetTunings(Kp, Ki, Kd);
  } else if(strcmp(topic, mqttTopicInSetKi) == 0) {
    Ki = payloadToFloat(payload, length, Ki);
    heaterPID.SetTunings(Kp, Ki, Kd);
  } else if(strcmp(topic, mqttTopicInSetKd) == 0) {
    Kd = payloadToFloat(payload, length, Kd);
    heaterPID.SetTunings(Kp, Ki, Kd);
  } else if(strcmp(topic, mqttTopicInSetFan) == 0) {
    int setting = payloadToInt(payload, length, fanSetting);
    if (setting < 0) fanSetting = 0;
    else if (setting > 100) fanSetting = 100;
    else fanSetting = setting;
    fan.setDutyCycle(fanSetting);
    Serial.print("Fan setting input = ");
    Serial.print(setting);
    Serial.print("   Calculated to: ");
    Serial.println(fanSetting);
  } else if(strcmp(topic, mqttTopicInReadSettings) == 0) {
    readSettings = true;
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

