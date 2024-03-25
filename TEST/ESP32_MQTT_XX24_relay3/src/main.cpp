#include <Arduino.h>
#include <myCredentials.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define relayPin1 32
#define relayPin2 33
#define relayPin3 25
#define relayPin4 26
#define relayPin5 27
#define relayPin6 14

// network and WiFi configuration
const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,24);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

// MQTT config
const char* mqttUser = "hansa"; //myCredentials::mqttUser
const char* mqttPassword= "Hkglape8266"; //myCredentials::password;

const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;

const char* topic1 = "ESPxx24/relay3/out1";
const char* topic2 = "ESPxx24/relay3/out2";
const char* topic3 = "ESPxx24/relay3/out3";
const char* topic4 = "ESPxx24/relay3/out4";
const char* topic5 = "ESPxx24/relay3/out5";
const char* topic6 = "ESPxx24/relay3/out6";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    
    if (mqttClient.connect("ESPClient_xx24" , mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");

      mqttClient.subscribe(topic1);
      mqttClient.subscribe(topic2);
    } else {
      Serial.print("Failed to connect to broker, rc = ");
      Serial.print(mqttClient.state());
      Serial.println(", retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void callBack(char* topic, byte* message, unsigned int length) {

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". message: ");

  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

//********** subscribing on out1 *******************
  if (String(topic) == "ESPxx24/relay3/out1") {
    Serial.print("Changing output1 on Relay to: ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(relayPin1, LOW);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(relayPin1, HIGH);
    }
  }
}
void setup() {
  Serial.begin(115200);

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);
  pinMode(relayPin5, OUTPUT);
  pinMode(relayPin6, OUTPUT);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);
  digitalWrite(relayPin3, HIGH);
  digitalWrite(relayPin4, HIGH);
  digitalWrite(relayPin5, HIGH);
  digitalWrite(relayPin6, HIGH);
  
  WiFi.config(staticIP,gateway,subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println("Connecting to WiFi.....");

  // Connect to MQTT broker
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(callBack);
    
  }
    Serial.print("Connected to WiFi. Local IP is: ");
    Serial.println(WiFi.localIP());


}

void loop() {

  while (!mqttClient.connected()) {
    reConnect();
  }

  mqttClient.loop();

}



