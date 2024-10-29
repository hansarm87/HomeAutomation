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

const char* mqttTopic1 = "ESPxx25/rollerBlinds/motor1";
const char* mqttTopic2 = "ESPxx25/rollerBlinds/motor2";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

//****************** DC MOTORS CONFIG *********************************************
/*
   Board pin | NodeMCU GPIO |  Arduino IDE
      A-           1             5 or D1
      A+           3             0 or D3
      B-           2             4 or D2
      B+           4             2 or D4
*/

#define MOTOR1_POWER D1 
#define MOTOR1_DIRECTION D3

#define MOTOR2_POWER D2 
#define MOTOR2_DIRECTION D4


// drivePower sets how fast the curtains will roll
// Can be set between 0 and 1023 (although blinds problaly wont move if values are too low)
int motor_drivePower = 1000;
//int motor_drivePower = 500; add an addtional if you want to control 2 motors at the same time

// driveDirection sets what direction the motor turns
uint8_t motor1_driveDirection = HIGH;
uint8_t motor2_driveDirection = HIGH;


// functions for roller blinds
void motor1Stop() {
    analogWrite(MOTOR1_POWER, 0);
    digitalWrite(MOTOR1_DIRECTION, 0); 
}
void motor2Stop() {
    analogWrite(MOTOR2_POWER, 0);
    digitalWrite(MOTOR2_DIRECTION, 0);  
}

void motor1GoUp() {
    analogWrite(MOTOR1_POWER, motor_drivePower);
    digitalWrite(MOTOR1_DIRECTION, motor1_driveDirection);
}

void motor1goDown() {
    analogWrite(MOTOR1_POWER, motor_drivePower);
    digitalWrite(MOTOR1_DIRECTION, !motor1_driveDirection);
}

void motor1goDownFor1Sec() {
    analogWrite(MOTOR1_POWER, motor_drivePower);
    digitalWrite(MOTOR1_DIRECTION, !motor1_driveDirection);
    delay(1000);
    motor1Stop();
}

void motor1goUpFor1Sec() {
    analogWrite(MOTOR1_POWER, motor_drivePower);
    digitalWrite(MOTOR1_DIRECTION, motor1_driveDirection);
    delay(1000);
    motor1Stop();
}
void motor2GoUp() {
    analogWrite(MOTOR2_POWER, motor_drivePower);
    digitalWrite(MOTOR2_DIRECTION, motor2_driveDirection);
}

void motor2goDown() {
    analogWrite(MOTOR2_POWER, motor_drivePower);
    digitalWrite(MOTOR2_DIRECTION, !motor2_driveDirection);
}

void motor2goDownFor1Sec() {
    analogWrite(MOTOR2_POWER, motor_drivePower);
    digitalWrite(MOTOR2_DIRECTION, !motor2_driveDirection);
    delay(1000);
    motor2Stop();
}

void motor2goUpFor1Sec() {
    analogWrite(MOTOR2_POWER, motor_drivePower);
    digitalWrite(MOTOR2_DIRECTION, motor2_driveDirection);
    delay(1000);
    motor2Stop();
}
void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    delay(500);

    if (mqttClient.connect("ESPClient_xx25", mqttUser, mqttPassword)){
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
//********** subscribing on motor 1 *******************
  if (String(topic) == "ESPxx25/rollerBlinds/motor1") {
    //Serial.print("Changing output1 on Relay2 to: ");

    if (messageTemp == "down") {
      motor1goDown();
      }
    else if (messageTemp == "up") {
      motor1GoUp();
      }  
    else if (messageTemp == "stop") {
      motor1Stop();
      }  
  }

//********** subscribing on motor 2*******************
  if (String(topic) == "ESPxx25/rollerBlinds/motor2") {
    //Serial.print("Changing output1 on Relay2 to: ");
    if (messageTemp == "down") {
      motor2goDown();
      }
    else if (messageTemp == "up") {
      motor2GoUp();
      }  
    else if (messageTemp == "stop") {
      motor2Stop();
      }  
  }

}



void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR1_POWER, OUTPUT);     
  pinMode(MOTOR2_POWER, OUTPUT);
  pinMode(MOTOR1_DIRECTION, OUTPUT);
  pinMode(MOTOR2_DIRECTION, OUTPUT);
  
  
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

