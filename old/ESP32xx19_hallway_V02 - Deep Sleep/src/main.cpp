#include <Arduino.h>
#include <myCredentials.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Pin Definitions
const int relayPin1 = 16;  // GPIO 16
const int relayPin2 = 17;  // GPIO 17
const int pirPin = 15;     // GPIO 15 (RTC_GPIO13)

// State Variables
boolean startTimer;
volatile bool motionDetected = false;
volatile bool bGoToSleep = false;

unsigned long currentTime;
unsigned long previousTime = 0;
volatile unsigned long counter = 0;
unsigned long counterCurrentTime;
unsigned long counterPreviousTime = 0;

// Wi-Fi and MQTT Credentials
const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;
IPAddress staticIP(192, 168, 0, 19);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
const char* mqttUser = "hansa";
const char* mqttPassword = "Hkglape8266";
const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;
const char* topic1 = "ESPxx19/relay/out1";

// Wi-Fi and MQTT Client Setup
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Reconnect to MQTT Broker
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

// Check for Motion Timeout
void checkForMotionTimeout() {
    currentTime = millis();
    if ((currentTime - previousTime > (60 * 1000)) && startTimer) {
        previousTime = currentTime;
        digitalWrite(relayPin1, HIGH);  // Turn off the relay
        Serial.println("Motion stopped");

        if (mqttClient.connected()) {
            bool publish = mqttClient.publish(topic1, "no motion detected! relay, out 1: off");
            Serial.print("Publish result: ");
            Serial.println(publish ? "success" : "failure");
        } else {
            Serial.println("MQTT not connected");
        }

        mqttClient.loop();
        delay(100);  // Ensure message is sent before sleeping
        startTimer = false;
        bGoToSleep = true;
    }

    counterCurrentTime = millis();
    if ((counterCurrentTime - counterPreviousTime > 1000) && startTimer) {
        counter++;
        Serial.print("Seconds since last motion: ");
        Serial.println(counter);
        counterPreviousTime = counterCurrentTime;
    }
}

// Go to Deep Sleep
void goToSleep() {
    bGoToSleep = false;
    Serial.println("Entering deep sleep...");
    esp_sleep_enable_ext0_wakeup((gpio_num_t)pirPin, 1); // Wake up when PIR pin is HIGH
    esp_deep_sleep_start();
}

// Publish Motion Detected Message
void PublishOnMotion() {
    Serial.println("Motion detected");
    Serial.println("Relay, out1: on");
    if (mqttClient.connected()) {
        bool publish = mqttClient.publish(topic1, "motion detected! relay, out 1: on");
        Serial.print("Publish result: ");
        Serial.println(publish ? "success" : "failure");
    } else {
        Serial.println("MQTT not connected");
    }
    motionDetected = false;
    delay(25);
}

// Interrupt Service Routine for Motion Detection
IRAM_ATTR void detectMovement() {
    previousTime = millis();
    motionDetected = true;
    startTimer = true;
    counter = 0;
}

// Setup Function
void setup() {
    Serial.begin(115200);

    pinMode(relayPin1, OUTPUT);
    pinMode(relayPin2, OUTPUT);
    digitalWrite(relayPin1, HIGH);
    digitalWrite(relayPin2, HIGH);

    pinMode(pirPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pirPin), detectMovement, RISING);

    // Check if waking up from deep sleep
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Waking up from deep sleep due to PIR sensor");
        
    }

    WiFi.config(staticIP, gateway, subnet);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.print("Connected to WiFi, IP is: ");
    Serial.println(WiFi.localIP());

    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setKeepAlive(60);

    Serial.print("WiFi Signal Strength (RSSI): ");
    Serial.println(WiFi.RSSI());

    reConnect();
}

// Main Loop Function
void loop() {
    int mqttConnectionAttempts = 0;
    while (!mqttClient.connected() && mqttConnectionAttempts < 3) {
        Serial.println("MQTT disconnected. Handling disconnection...");
        reConnect();
        if (!mqttClient.connected() && mqttConnectionAttempts >= 3) {
            Serial.println("Restarting the ESP...");
            ESP.restart();
        } else {
            mqttClient.loop();
        }
    }

    if (motionDetected) {
        PublishOnMotion();
    }

    if (startTimer) {
        digitalWrite(relayPin1, LOW);
    }

    checkForMotionTimeout();

    if (bGoToSleep) {
        goToSleep();
    }
}
