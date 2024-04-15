#include <Arduino.h>
#include <myCredentials.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>


//WiFi config
const char* SSID = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,21);
IPAddress gateway(0,0,0,0);
IPAddress subnet(255,255,255,0);

//MQTT config
const char* mqttUser = myCredentials::mqttUser;
const char* mqttPassword = myCredentials::mqttPassword;

const int mqttPort = 1883;
const char* mqttServer = "192.168.0.246";

const char* mqttTopic1 = "ESPxx21/rollerBlindMotor";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

//****************** DC MOTORS CONFIG *********************************************

#define LEFT_MOTORS_POWER D1 
#define LEFT_MOTORS_DIRECTION D3

//#define RIGHT_MOTORS_POWER D2 // Motor B = right wheels
//#define RIGHT_MOTORS_DIRECTION D4


// drivePower sets how fast the curtains will roll
// Can be set between 0 and 1023 (although blinds problaly wont move if values are too low)
int left_drivePower = 1000;
//int right_drivePower = 500;

// driveDirection sets what direction the motor turns
uint8_t left_driveDirection = HIGH;
//uint8_t right_driveDirection = HIGH;


// functions for roller blinds
void stop() {
    analogWrite(LEFT_MOTORS_POWER, 0);
    digitalWrite(LEFT_MOTORS_DIRECTION, 0);
}

void goUp() {
    analogWrite(LEFT_MOTORS_POWER, left_drivePower);
    digitalWrite(LEFT_MOTORS_DIRECTION, left_driveDirection);
}

void goDown() {
    analogWrite(LEFT_MOTORS_POWER, left_drivePower);
    digitalWrite(LEFT_MOTORS_DIRECTION, !left_driveDirection);
}

void goDownFor1Sec() {
    analogWrite(LEFT_MOTORS_POWER, left_drivePower);
    digitalWrite(LEFT_MOTORS_DIRECTION, !left_driveDirection);
    delay(1000);
    stop();
}

void goUpFor1Sec() {
    analogWrite(LEFT_MOTORS_POWER, left_drivePower);
    digitalWrite(LEFT_MOTORS_DIRECTION, left_driveDirection);
    delay(1000);
    stop();
}

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    delay(500);

    if (mqttClient.connect("ESPClient_XX21", mqttUser, mqttPassword)){
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(mqttTopic1);
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
  if (String(topic) == "ESPxx21/rollerBlindMotor") {
    //Serial.print("Changing output1 on Relay2 to: ");
    int timer = 0;

    if (messageTemp == "down") {
      //Serial.println("Changing output1 on Relay2 to: 100% on");
      while (timer < 20) {
        goDownFor1Sec();
        timer += 1;
        Serial.println(timer);
      }
    Serial.println("Roller blind are down"); 
      
    }
    else if (messageTemp == "up") {
      while (timer < 20) {
        goUpFor1Sec();
        timer += 1;
        Serial.println(timer);
      }
    Serial.println("Roller blind are up");
    }  

  }
}



void setup() {
  Serial.begin(115200);
  
  pinMode(LEFT_MOTORS_POWER, OUTPUT);     
  //pinMode(RIGHT_MOTORS_POWER, OUTPUT);
  pinMode(LEFT_MOTORS_DIRECTION, OUTPUT);
  //pinMode(RIGHT_MOTORS_DIRECTION, OUTPUT);
  
  
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

