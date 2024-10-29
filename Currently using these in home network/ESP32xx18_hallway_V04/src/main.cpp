#include <Arduino.h>
#include <WiFi.h>
#include <myCredentials.h>
#include <esp_sleep.h>
#include <PubSubClient.h>

// PIR Sensor and Timing Configuration
const int pirPin = 15; // Pin connected to the PIR sensor
const unsigned long sleepInterval = 50 * 1000; // 50 seconds before sleep
const unsigned long debounceTime = 5000; //  seconds debounce time
const unsigned long debugInterval = 1000; //  seconds interval for debugging

// Timing Variables
unsigned long currentTime = 0;
unsigned long lastMotionDetectedTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastDebugPublishTime = 0;
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

// Function to Connect to WiFi
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

// Function to Reconnect to MQTT Broker
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

// Function to Publish MQTT Messages with Confirmation
bool publishMQTTMessage(const char* topic, const char* message) {
  int retryCount = 0;
  while (!mqttClient.publish(topic, message) && retryCount < 3) {
    Serial.println("Publish failed, retrying...");
    MQTTreconnect();
    retryCount++;
    delay(100);
  }

  // Confirm if the message was sent
  if (retryCount < 3) {
    Serial.println("Publish successful");
    return true;
  } else {
    Serial.println("Publish failed after retries");
    return false;
  }
}

// Function to Check for Motion Timeout
void checkForMotionTimeout() {
  currentTime = millis();
  mqttClient.loop(); // Maintain MQTT connection

  if (startTimer && (currentTime - lastMotionDetectedTime >= sleepInterval)) {
    Serial.print("No motion detected for ");
    Serial.print(sleepInterval / 1000);  // Convert milliseconds to seconds
    Serial.println(" seconds.");
    
    // Calculate the number of seconds before sleep
    unsigned long secondsBeforeSleep = (currentTime - lastMotionDetectedTime) / 1000;
    String sleepMessage = "Going to sleep after " + String(secondsBeforeSleep) + " seconds of no motion.";
    Serial.println(sleepMessage);
    
    // Send the number of seconds passed before sleep to the new MQTT topic
    publishMQTTMessage("ESPxx18/debug/sleep_time", sleepMessage.c_str());
    
    // Publish motion stopped message and go to sleep
    if (mqttClient.connected()) {
      if (publishMQTTMessage("ESPxx18/pir", "motion stopped")) {
        Serial.println("Going to sleep...");
        delay(100); // Allow time for message transmission
        esp_deep_sleep_start();
      } else {
        Serial.println("Failed to send MQTT message, reconnecting...");
        MQTTreconnect();
      }
    } else {
      Serial.println("MQTT not connected, retrying...");
      MQTTreconnect();
    }
  }
}

// Function to send timing debug messages
void sendDebugTiming() {
  currentTime = millis();

  if (currentTime - lastDebugPublishTime >= debugInterval) {
    String message = "No motion for " + String((currentTime - lastMotionDetectedTime) / 1000) + " seconds.";
    Serial.println(message);
    publishMQTTMessage("ESPxx18/debug/timing", message.c_str());
    lastDebugPublishTime = currentTime;
  }
}

// Function to check if the PIR sensor is stable before declaring motion
bool isMotionDetected() {
  int sensorReadings = 0;
  for (int i = 0; i < 5; i++) {  // Read the sensor 5 times
    if (digitalRead(pirPin) == HIGH) {
      sensorReadings++;
    }
    delay(50);  // Small delay between readings
  }
  return (sensorReadings >= 3);  // If 3 or more readings are HIGH, consider it valid
}

void handleWakeup() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal (PIR sensor)");

      // Ensure WiFi and MQTT are connected
      if (!mqttClient.connected()) {
        MQTTreconnect();  // Try reconnecting to MQTT
      }

      // Publish the motion detected message
      if (mqttClient.connected()) {
        publishMQTTMessage("ESPxx18/pir", "motion detected");
      } else {
        Serial.println("Failed to reconnect to MQTT after wake-up");
      }

      // Reset motion detection timer
      lastMotionDetectedTime = millis();
      startTimer = true;
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      // Handle timer wake-up if needed
      break;

    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      Serial.println("ESP32 just powered up or woke up for another reason");
      
      // Ensure WiFi and MQTT are connected after power-up
      if (!mqttClient.connected()) {
        MQTTreconnect();
      }
      break;
  }
}

// Setup Function
void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT_PULLUP);

  // Configure the PIR sensor to be used as a wake-up source
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 1); // High level triggers wake-up

  // Initialize WiFi and MQTT Connections
  connectToWiFi();
  mqttClient.setServer(mqttServer, mqttPort);
  MQTTreconnect();

  // Check why the ESP32 woke up
  handleWakeup();  // Handle the wake-up logic here
  
  // Optional: Small stabilization delay to allow system to settle
  unsigned long stabilizationStartTime = millis();
  while (millis() - stabilizationStartTime < 500) {
    mqttClient.loop(); // Keep MQTT connection alive during stabilization
  }
/* Check the Wake-Up Reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Wakeup caused by external signal (PIR sensor)");
    if (mqttClient.connected()) {
      publishMQTTMessage("ESPxx18/pir", "motion detected");
    } else {
      MQTTreconnect();
    }
    lastMotionDetectedTime = millis();
    startTimer = true;
  } else {
    Serial.println("ESP32 just powered up or woke up for another reason");
  }
  delay(500); // Allow system to stabilize
  */
  
}

// Main Loop
void loop() {
  currentTime = millis();
  mqttClient.loop(); // Maintain MQTT connection

  // Check PIR Sensor for Motion
  if (isMotionDetected() && (currentTime - lastMotionDetectedTime > debounceTime))  {
    Serial.println("Motion detected, resetting timer...");
    if (mqttClient.connected()) {
      publishMQTTMessage("ESPxx18/pir", "motion detected");
    } else {
      MQTTreconnect();
    }
    lastMotionDetectedTime = currentTime;
    startTimer = true;
  }

  // Check if Motion Timeout has been Reached
  if (startTimer) {
    checkForMotionTimeout();
  }

  // Send debug messages to track time since last motion
  sendDebugTiming();
}
