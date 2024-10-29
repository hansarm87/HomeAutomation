#include <Arduino.h>
#include <myCredentials.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// WiFi config
const char* SSID = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192, 168, 0, 25);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT config
const char* mqttUser = myCredentials::mqttUser;
const char* mqttPassword = myCredentials::mqttPassword;
const int mqttPort = 1883;
const char* mqttServer = "192.168.0.246";

const char* mqttTopic1 = "ESPxx25/rollerBlinds/motor1";
const char* mqttTopic2 = "ESPxx25/rollerBlinds/motor2";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// DC Motors Configuration
#define MOTOR1_POWER D1 
#define MOTOR1_DIRECTION D3
#define MOTOR2_POWER D2 
#define MOTOR2_DIRECTION D4

int motor_drivePower = 1000;  // Set between 0 and 1023

// Functions for motor control
void motorStop(int powerPin, int directionPin) {
  analogWrite(powerPin, 0);
  digitalWrite(directionPin, LOW);
}

void motorMove(int powerPin, int directionPin, uint8_t direction) {
  analogWrite(powerPin, motor_drivePower);
  digitalWrite(directionPin, direction);
}

void controlMotor(int motor, String command) {
  int powerPin, directionPin;
  
  // Select motor pins based on motor number
  if (motor == 1) {
    powerPin = MOTOR1_POWER;
    directionPin = MOTOR1_DIRECTION;
  } else if (motor == 2) {
    powerPin = MOTOR2_POWER;
    directionPin = MOTOR2_DIRECTION;
  } else {
    Serial.println("Invalid motor number. No action taken.");
    return;
  }

  // Execute command
  if (command == "up") {
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.println(" going up...");
    motorMove(powerPin, directionPin, HIGH);
  } else if (command == "down") {
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.println(" going down...");
    motorMove(powerPin, directionPin, LOW);
  } else if (command == "stop") {
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.println(" stopping...");
    motorStop(powerPin, directionPin);
  } else {
    Serial.println("Unknown command. No action taken.");
  }
}

// MQTT callback function
void callBack(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Compare topics and control motors
  if (strcmp(topic, mqttTopic1) == 0) {
    Serial.println("Processing command for Motor 1...");
    controlMotor(1, messageTemp);
  } else if (strcmp(topic, mqttTopic2) == 0) {
    Serial.println("Processing command for Motor 2...");
    controlMotor(2, messageTemp);
  } else {
    Serial.println("Unknown topic. No action taken.");
  }
}

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    delay(500);
    if (mqttClient.connect("ESPClient_xx25", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(mqttTopic1);
      mqttClient.subscribe(mqttTopic2);
    } else {
      Serial.print("Failed to connect to MQTT broker. RC: ");
      Serial.print(mqttClient.state());
      Serial.println(", Retrying in 5 seconds");
      delay(5000);
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
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callBack);
}

void loop() {
  if (!mqttClient.connected()) {
    reConnect();
  }
  mqttClient.loop();
}
