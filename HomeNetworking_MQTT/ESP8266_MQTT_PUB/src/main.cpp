#include <Arduino.h>
#include <myCredentials.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//WiFi config:

const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,24); 
IPAddress gateway(0,0,0,0);
IPAddress subnet(255,255,255,0);

//MQTT config

const char* mqttUser = myCredentials::mqttUser;
const char* mqttPassword = myCredentials::mqttPassword;

const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;

const char* mqttTopic1 = "ESPxxxx/relay1/hallwayLamp";
const char* mqttTopic2 = "ESPxxxx/relay1/livingroomLamp";

WiFiClient ESPClient;
PubSubClient mqttClient(ESPClient);

// variables for pushbuttons
const int PB1 = D2;
int PB1Old = HIGH;  // Assuming pull-down resistor, so initial state is HIGH
int PB1New;
int PB1State = 0;  // Global variable to remember button state



const int PB2 = D3;
int PB2Old = HIGH;
int PB2New;
int PB2State = 0;

// variables for Pir and timer (ISR) function
const int pirPin = D1;
boolean startTimer;


unsigned long currentTime;
unsigned long previousTime = 0;

volatile unsigned long counter = 0;
unsigned long counterCurrentTime;
unsigned long counterPreviousTime = 0;

//*********** MQTT **************

void reConnect() {

  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    delay(500);

    if (mqttClient.connect("ESPClient_XXXXX", mqttUser, mqttPassword)){
      Serial.println("Connected to MQTT broker");
    }
    else {
      Serial.print("Failed to connect to MQTT broker. RC : ");
      Serial.print(mqttClient.state());
      Serial.println(", Retrying in 5 seconds");
      delay(5000);
    }
  }
  
}

//*********** ISR ***************

IRAM_ATTR void detectMovement() {
  Serial.println("Motion detected");
  previousTime = millis();
  startTimer = true;
  counter = 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);

  pinMode(pirPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pirPin), detectMovement, RISING);

  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);
  

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(1000);
  }

  Serial.print("Connected to WiFi! IP address is: ");
  Serial.println(WiFi.localIP());
  Serial.println("WiFi status: " + String(WiFi.status()));

  mqttClient.setServer(mqttServer, mqttPort);

}

void loop() {
  while (!mqttClient.connected()) {
    reConnect();
  }

  mqttClient.loop();

//******************** publish on PB1 (hallwayLamp) **********************
  PB1New = digitalRead(PB1);
  
  if (PB1Old == 0 && PB1New == 1) {
    Serial.println("PB1 Pressed!");
    if ( PB1State == 0) {
      Serial.println("Sending PB1 state = on ");
      mqttClient.publish(mqttTopic1, "100%_on");
      PB1State = 1;
    }
    else {
      Serial.println("Sending PB1 state = off ");
      mqttClient.publish(mqttTopic1, "off");
      PB1State = 0;
    }
  }
  PB1Old = PB1New;
//******************** publish on PB2 (livingroomLamp) **********************
  PB2New = digitalRead(PB2);

  if (PB2Old == 0 && PB2New == 1) {
    Serial.println("PB2 pressed!");
    if (PB2State == 0) {
      Serial.println("Sending PB2 State = on");
      mqttClient.publish(mqttTopic2, "on");
      PB2State = 1;
    }
    else {
      Serial.println("Sending PB2 State = off");
      mqttClient.publish(mqttTopic2, "off");
      PB2State = 0;
    }
  }
  PB2Old = PB2New;
  
//******************** timer function for deactivating hallwayLamp **********************
currentTime = millis();
if ((currentTime - previousTime > (50 * 1000)) && startTimer) {
  previousTime = currentTime;
  mqttClient.publish(mqttTopic1, "off");
  Serial.println("Motion stopped");
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
  mqttClient.publish(mqttTopic1, "100%_on");
}




  /*Serial.print(" PB1 (hallwayLamp) state: ");
  Serial.println(PB1State);
  
  Serial.print(" PB2 (livingroomLamp) state: ");
  Serial.println(PB2State);
  */

  
 
 delay(100);
}
