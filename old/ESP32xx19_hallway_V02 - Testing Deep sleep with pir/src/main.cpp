#include <Arduino.h>
#include <WiFi.h>
#include <myCredentials.h>
#include <esp_sleep.h>
#include <PubSubClient.h>

const int pirPin = 15;   // Pin connected to the PIR sensor (RTC_GPIO13)
const unsigned long sleepInterval = 59 * 1000;  // 59 seconds
const unsigned long gracePeriod = 500; // 500 ms grace period
const unsigned long debounceTime = 3000; // 3 seconds debounce time

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long lastMotionDetectedTime = 0;
unsigned long counter = 0;
unsigned long counterPreviousTime = 0;

bool startTimer = false;

// WiFi and MQTT Credentials
const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;
IPAddress staticIP(192, 168, 0, 18);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
const char* mqttUser = "hansa";
const char* mqttPassword = "Hkglape8266";
const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void connectToWiFi() {
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Attempting to connect to WiFi...");
  }
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}

void MQTTreconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect("ESPClient_xx18", mqttUser, mqttPassword)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(mqttClient.state());
      delay(1000);
    }
  }
}

void checkForMotionTimeout() {
  currentTime = millis();

  if (startTimer && (currentTime - previousTime > sleepInterval)) {
    Serial.println("No motion detected for 60 seconds.");
    if (mqttClient.connected()) {
      bool publish = mqttClient.publish("ESPxx18/pir", "No motion detected for 60 seconds");
      Serial.print("Publish result: ");
      Serial.println(publish ? "Success" : "Failure");

      mqttClient.loop();
      delay(100);  // Ensure message is sent

      if (publish) {
        Serial.println("Going to sleep...");
        esp_deep_sleep_start();
      } else {
        Serial.println("failed to send mqtt message, reconnecting..");
        MQTTreconnect();
      }
      

    } else {
      Serial.println("MQTT not connected, retrying...");
      MQTTreconnect();
    }

    
  }

  // Display seconds since last motion
  if (startTimer && (currentTime - counterPreviousTime >= 1000)) {
    counter++;
    Serial.print("Seconds since last motion: ");
    Serial.println(counter);
    counterPreviousTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT_PULLUP);

  // Configure the PIR sensor to be used as a wake-up source
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 1);  // High level triggers wake-up

  connectToWiFi();
  mqttClient.setServer(mqttServer, mqttPort);
  MQTTreconnect();

  // Check the wake-up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Wakeup caused by external signal (PIR sensor)");
    if (mqttClient.connected()) {
      mqttClient.publish("ESPxx18/pir", "Motion detected! ESP woke up");
    } else {
      MQTTreconnect();
    }
    previousTime = millis();
    startTimer = true;
  } else {
    Serial.println("ESP32 just powered up or woke up for another reason");
  }

  delay(gracePeriod);
}

void loop() {
  currentTime = millis();

  if (digitalRead(pirPin) == HIGH && (currentTime - lastMotionDetectedTime > debounceTime)) {
    Serial.println("Motion detected, resetting timer...");
    if (mqttClient.connected()) {
      mqttClient.publish("ESPxx18/pir", "Motion detected! Timer started");
    } else {
      MQTTreconnect();
    }
    mqttClient.loop();
    delay(100);  // Ensure message is sent

    previousTime = millis();
    startTimer = true;
    counter = 0;
    lastMotionDetectedTime = currentTime;
  }

  if (startTimer) {
    checkForMotionTimeout();
  }
}
