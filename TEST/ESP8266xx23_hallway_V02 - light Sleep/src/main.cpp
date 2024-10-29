#include <Arduino.h>
#include <myCredentials.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const int relayPin1 = D5;
const int relayPin2 = D6;

const int pirPin = D2;
boolean startTimer;
volatile bool motionDetected = false;  // Declare this globally

unsigned long currentTime;
unsigned long previousTime = 0;
volatile unsigned long counter = 0;
unsigned long counterCurrentTime;
unsigned long counterPreviousTime = 0;

const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,19);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

const char* mqttUser = "hansa";
const char* mqttPassword = "Hkglape8266";
const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;

const char* topic1 = "ESPxx19/relay/out1";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void reConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect("ESPClient_xx19", mqttUser, mqttPassword)) {
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
void checkForMotionTimeout() {
    currentTime = millis();
    if ((currentTime - previousTime > (60 * 1000)) && startTimer) {
        previousTime = currentTime;
        digitalWrite(relayPin1, HIGH);  // Turn off the relay
        Serial.println("Motion stopped");
        mqttClient.publish(topic1, "no motion detected! relay, out 1: off");
        startTimer = false;

        // Enter light sleep after motion stops
        Serial.println("Entering light sleep...");
        WiFi.forceSleepBegin();
        delay(1);  // Short delay to allow sleep to start
    }

    counterCurrentTime = millis();
    if ((counterCurrentTime - counterPreviousTime > 1000) && startTimer) {
        counter++;
        Serial.print("Seconds since last motion: ");
        Serial.println(counter);
        counterPreviousTime = counterCurrentTime;
    }
}

void PublishOnMotion() {
  Serial.println("Motion detected");
  Serial.println("Relay, out1: on");
  mqttClient.publish(topic1, "motion detected! relay, out 1: on");
}

IRAM_ATTR void detectMovement() {
  previousTime = millis();
  motionDetected = true;  // Set the flag
  startTimer = true;
  counter = 0;
  // Wake up from light sleep
  WiFi.forceSleepWake();
  delay(1);
}

void setup() {
  Serial.begin(115200);

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);

  pinMode(pirPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pirPin), detectMovement, RISING);

  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected to WiFi, IP is: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setKeepAlive(60);  // Set the keep-alive interval to 60 seconds

  Serial.print("WiFi Signal Strength (RSSI): ");
  Serial.println(WiFi.RSSI());

  // Attempt to connect to MQTT broker once during setup
  reConnect();
}

void loop() {
  
    int mqttConnectionAttempts = 0;
    while (!mqttClient.connected() && mqttConnectionAttempts < 3) {
       Serial.println("MQTT disconnected. Handling disconnection...");
      reConnect();
      if (!mqttClient.connected() && mqttConnectionAttempts > 3) {
        Serial.println("restarting the ESP...");
        ESP.restart();
    } else {
      mqttClient.loop();
      mqttClient.setKeepAlive(60);  // Set the keep-alive interval to 60 seconds
    }
    } 


  if (motionDetected) {
        PublishOnMotion();  // Call the function to handle motion
        motionDetected = false;  // Reset the flag
    }

  if (startTimer == true) {
    digitalWrite(relayPin1, LOW);
  }

  checkForMotionTimeout();
}
