#include <Arduino.h>
#include <myCredentials.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const int relayPin1 = D5;
const int relayPin2 = D6;

// network and WiFi configuration
const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,19);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

// MQTT config
const char* mqttUser = "hansa"; //myCredentials::mqttUser
const char* mqttPassword= "Hkglape8266"; //myCredentials::password;

const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;

const char* topic = "ESPxx18/pir";
const char* topic1 = "ESPxx19/relay/out1";
const char* topic2 = "ESPxx19/relay/out2";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    
    if (mqttClient.connect("ESPClient_xx19" , mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");

      mqttClient.subscribe(topic);
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
//********** subscribing on hallway lamp *******************
  if (String(topic) == "motion detected") {
    Serial.print("Changing output1 on Relay to: ");
    if (messageTemp == "on") {
      Serial.println("out 1 on");
      digitalWrite(relayPin1, LOW);
    }
    else if (messageTemp == "motion stopped") {
      Serial.println("out 1 off");
      digitalWrite(relayPin1, HIGH);
    }
    
  }
//********** subscribing on livingroom lamp *******************
  if (String(topic) == "ESPxxxx/relay/out2") {
    Serial.print("Changing output2 on relay to: ");
    if (messageTemp == "on") {
      Serial.println("out 2 on");
      digitalWrite(relayPin2, LOW);
    }
    else if (messageTemp == "off") {
    Serial.println("out 2 off");
    digitalWrite(relayPin2, HIGH);
  }
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);

  //connect to wifi
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
    Serial.print("Connected to WiFi, IP is: ");
    Serial.println(WiFi.localIP());
  
  // Connect to MQTT broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callBack);

}

void loop() {
  
  while (!mqttClient.connected()) {
    reConnect();
  }

  mqttClient.loop();

 
}

