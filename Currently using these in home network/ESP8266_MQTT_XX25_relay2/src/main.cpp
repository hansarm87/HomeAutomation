#include <Arduino.h>
#include <myCredentials.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>


//WiFi config
const char* SSID = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,25);
IPAddress gateway(0,0,0,0);
IPAddress subnet(255,255,255,0);

//MQTT config
const char* mqttUser = myCredentials::mqttUser;
const char* mqttPassword = myCredentials::mqttPassword;

const int mqttPort = 1883;
const char* mqttServer = "192.168.0.246";

const char* mqttTopic1 = "ESPxx25/relay2/floorLamp1";
const char* mqttTopic2 = "ESPxx25/relay2/floorLamp2";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

//pin declarations for relay output
const int OUT1 = D5;
const int OUT2 = D6;

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    delay(500);

    if (mqttClient.connect("ESPClient_XX25", mqttUser, mqttPassword)){
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(mqttTopic1);
      mqttClient.subscribe(mqttTopic2);
    }
    else {
      Serial.print("Failed to connect to MQTT broker. RC : ");
      Serial.print(mqttClient.state());
      Serial.println(", Retrying in 5 seconds");
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
  if (String(topic) == "ESPxx25/relay2/floorLamp1") {
    //Serial.print("Changing output1 on Relay2 to: ");
    if (messageTemp == "100%_on") {
      //Serial.println("Changing output1 on Relay2 to: 100% on");
      digitalWrite(OUT1, HIGH);
      delay(100);
      digitalWrite(OUT1, LOW);
    }
    else if (messageTemp == "off") {
      
      digitalWrite(OUT1, HIGH);
    }
    else if (messageTemp == "50%_on"){
      digitalWrite(OUT1, HIGH);
      delay(100);
      digitalWrite(OUT1, LOW);
      delay(100);
      digitalWrite(OUT1, HIGH);
      delay(100);
      digitalWrite(OUT1, LOW); 
    }
    else if (messageTemp == "25%_on") {
      digitalWrite(OUT1, HIGH);
      delay(100);
      digitalWrite(OUT1, LOW);
      delay(100);
      digitalWrite(OUT1, HIGH);
      delay(100);
      digitalWrite(OUT1, LOW);
      delay(100); 
      digitalWrite(OUT1, HIGH);
      delay(100);
      digitalWrite(OUT1, LOW);
    } 
  }

  //********** subscribing on livingroom lamp *******************
  if (String(topic) == "ESPxx25/relay2/floorLamp2") {
    Serial.print("Changing output2 on relay2 to: ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(OUT2, LOW);
    }
    else if (messageTemp == "off") {
    Serial.println("off");
    digitalWrite(OUT2, HIGH);
  }
  }

}

void setup() {
  Serial.begin(115200);
  
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(SSID, password);
  

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connecting...");
    delay(1000);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP address: ");
    Serial.print(WiFi.localIP());
  } else {
    Serial.println(WiFi.status());
  }
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

