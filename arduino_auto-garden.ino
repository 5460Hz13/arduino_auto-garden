//------------------WIFI------------------
#include <SPI.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status
//-----------------/WIFI------------------



//------------------DHT22------------------
// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 0     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
//-----------------/DHT22------------------

//------------------RELAY------------------
const int relayPin_lights = 1; //lights
const int relayPin_fan = 2; //fans
//-----------------/RELAY------------------
//------------------MQTT------------------
#include <ArduinoMqttClient.h>

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = BROKER_IP;
int        port     = BROKER_PORT;
const char temp_topic[]  = "grow_temp";
const char hum_topic[]  = "grow_hum";
const char relay_topic_lights[]  = "grow_light";
const char relay_topic_fan[]  = "grow_fan";
const char poti_topic[]  = "grow_poti";
const char info_topic[]  = "grow_info";
String info_msg = "no wifi";
const long interval = 30000;
unsigned long previousMillis = 0;

int count = 0;

char words[] = "";

//-----------------/MQTT------------------
//-------------POTI--------------
//#include <Wire.h>
//#define WIRE Wire2
#include <MCP4018-SOLDERED.h>

MCP4018_SOLDERED MCP;

bool potiAvailable;
//------------/POTI--------------


void setup() {
  Serial.begin(9600);
  
  //------------------SERIAL------------------
  //Initialize serial and wait for port to open:
  
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */
  //-----------------/SERIAL------------------
  //------------------DHT22------------------
  // Initialize device.
  dht.begin();
  

  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  //-----------------/DHT22------------------
  //------------------WIFI------------------
  

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  WiFiConnect();
  info_msg = "Wifi Connected";
  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();
  //-----------------/WIFI------------------
  //------------------RELAY------------------
  pinMode(relayPin_lights, OUTPUT);
  pinMode(relayPin_fan, OUTPUT);
  //-----------------/RELAY------------------
  //------------------MQTT------------------
  MQTTConnect();
  info_msg = "MQTT Connected";
  
  //-----------------/MQTT------------------
  //-------------POTI--------------
  info_msg = "Starting MCP";
  MCP.begin();
  info_msg = "Starting Poti Test";
  PotiTest();
  info_msg = "Finished Poti Test";

  //------------/POTI--------------
}

void loop() {
  //------------------WIFI------------------
  if(WiFi.status() != WL_CONNECTED){
      Serial.println("Wifi disconnected, attempting reset");
      WiFi.end();
      WiFiConnect();
      MQTTConnect();
  }
  //-----------------/WIFI------------------
  //-----------------DHT22------------------
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event.
  sensors_event_t event;
  //-----------------/DHT22------------------
  //------------------RELAY------------------

  //-----------------/RELAY------------------
  //-------------SOIL MOISTURE--------------
  /*
  moistVal = analogRead(moistPin);
  */
  //------------/SOIL MOISTURE--------------
  //------------------MQTT------------------
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  //--------------RECEIVE--------------
  /*
  //RELAY
    int messageSize = mqttClient.parseMessage();
    if (messageSize) {
      // we received a message, print out the topic and contents
      Serial.print("Received a message with topic '");
      Serial.print(mqttClient.messageTopic());
      Serial.print("The message is: ");
      // use the Stream interface to print the contents
      while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
      }
    }
  */

  //--------------SEND--------------
  // to avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;
    
    //TEMPERATURE
    dht.temperature().getEvent(&event);
    mqttClient.beginMessage(temp_topic);
    mqttClient.print(event.temperature);
    mqttClient.endMessage();

    //HUMIDITY
    dht.humidity().getEvent(&event);
    mqttClient.beginMessage(hum_topic);
    mqttClient.print(event.relative_humidity);
    mqttClient.endMessage();

    //INFO
    mqttClient.beginMessage(info_topic);
    mqttClient.print(info_msg);
    mqttClient.endMessage();

    Serial.println("Updating MQTT Broker");

    
    count++;
  }
  //-----------------/MQTT------------------
  
}
//------------------WIFI------------------
void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void WiFiConnect(){
  // attempt to connect to WiFi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);

    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
}
//-----------------/WIFI------------------
//-----------------MQTT------------------
void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");
  
  if(mqttClient.messageTopic() == relay_topic_lights){
    
    if(messageSize < 3){
      Serial.println("Relay on");
      digitalWrite(relayPin_lights, HIGH);
    }
    else{
      Serial.println("Relay off");
      digitalWrite(relayPin_lights, LOW);
    }
  }

  if(mqttClient.messageTopic() == relay_topic_fan){
    
    if(messageSize < 3){
      Serial.println("Relay on");
      digitalWrite(relayPin_fan, HIGH);
    }
    else{
      Serial.println("Relay off");
      digitalWrite(relayPin_fan, LOW);
    }
  }

  if(mqttClient.messageTopic() == poti_topic){
    int requestedPercent = messageSize;
    
    Serial.print("Setting Poti to ");
    Serial.print(requestedPercent);
    Serial.println("%");
    MCP.setWiperPercent(requestedPercent);
    Serial.print("Poti Set to: ");
    Serial.print(MCP.getWiperPercent());
    Serial.println("%");
  }
}

void MQTTConnect(){
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  //subscribe to topics
  mqttClient.subscribe(relay_topic_lights);
  mqttClient.subscribe(relay_topic_fan);
  mqttClient.subscribe(poti_topic);
  Serial.println("Waiting for messages on topic: ");
  Serial.println(relay_topic_lights);
  Serial.println(relay_topic_fan);
  Serial.println(poti_topic);
}
//-----------------/MQTT------------------
void PotiTest () {
  Serial.println("MCP4018 test begins:");
  Serial.print("Wiper ranges from: ");
  MCP.setWiperPercent(0);
  delay(2000);
  Serial.print(MCP.getWiperPercent());
  Serial.print("% to: ");
  MCP.setWiperPercent(100);
  delay(2000);
  Serial.println(MCP.getWiperPercent());
  Serial.println("and can be set to anything in-between.");
  MCP.setWiperPercent(0);
  Serial.println("MCP4018 test finished");
}