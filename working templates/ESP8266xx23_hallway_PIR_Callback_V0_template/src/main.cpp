#include <Arduino.h>
#include <myCredentials.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const int relayPin1 = D5;
const int relayPin2 = D6;


// variables for Pir and timer (ISR) function
const int pirPin = D2;
boolean startTimer;


unsigned long currentTime;
unsigned long previousTime = 0;

volatile unsigned long counter = 0;
unsigned long counterCurrentTime;
unsigned long counterPreviousTime = 0;


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

const char* topic1 = "ESPxx19/relay/out1"; // hallwaylamp


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    
    if (mqttClient.connect("ESPClient_xx19" , mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");

      mqttClient.subscribe(topic1);

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
  if (String(topic) == "ESPxxxx/relay/out1") {
    Serial.print("Changing output1 on Relay to: ");
    if (messageTemp == "") { // inser topic to listen to
       // insert commands
    }
    else if (messageTemp == "") {
      
    }
    else if (messageTemp == ""){
    }
  }
//********** subscribing on livingroom lamp *******************
  if (String(topic) == "") {
    Serial.print("Changing output2 on relay to: ");
    if (messageTemp == "") {
      Serial.println("");
      digitalWrite(relayPin2, LOW);
    }
    else if (messageTemp == "") {
    Serial.println("");
    digitalWrite(relayPin2, HIGH);
  }
  }
  
}

//*********** ISR ***************

IRAM_ATTR void detectMovement() {
  Serial.println("Motion detected");
  Serial.println("Relay, out1: on");
  mqttClient.publish(topic1, "motion detected! relay, out 1: on");
  previousTime = millis();
  startTimer = true;
  counter = 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);

  pinMode(pirPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pirPin), detectMovement, RISING);

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
  //mqttClient.setCallback(callBack);

}

void loop() {
  
  while (!mqttClient.connected()) {
    reConnect();
  }

  mqttClient.loop();

  //******************** timer function for deactivating hallwayLamp **********************
currentTime = millis();
if ((currentTime - previousTime > (60 * 1000)) && startTimer) {
  previousTime = currentTime;
  digitalWrite(relayPin1,HIGH);
  Serial.println("Motion stopped");
  mqttClient.publish(topic1, "no motion detected! relay, out 1: off");
  startTimer = false;
}
counterCurrentTime = millis();
if ((counterCurrentTime - counterPreviousTime > 1000) && startTimer) {
    counter++;
    Serial.print("Seconds since last motion: ");
    Serial.println(counter);
    counterPreviousTime = counterCurrentTime;
}
if (startTimer==true) {
  digitalWrite(relayPin1,LOW);
  
}
}

