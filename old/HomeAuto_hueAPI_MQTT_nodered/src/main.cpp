#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <myCredentials.h>

// network and WiFi configuration
const char* ssid = myCredentials::ssid;
const char* password = myCredentials::password;

IPAddress staticIP(192,168,0,23);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

//Philips hue API config
const char* hueBridgeIP = "192.168.0.2";
const char* hueUser = myCredentials::hueUser;

// MQTT broker config
const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;
const char* mqttUser = myCredentials::mqttUser;
const char* mqttPassword = myCredentials::mqttPassword;


// pinout declaration for relays
const int relayPin1 = D1;
const int relayPin2 = D2;

// variables for timer functionality

boolean StartTimer = false;


// Object used for handling the WiFi connection in regards to MQTT
WiFiClient wifiClient; 

// The mqtt object requires a WiFiClient instance (wifiClient) to establish a connection to an MQTT broker
PubSubClient mqttClient(wifiClient); 

// Instantiate another WiFiClient object named hueApiClient. 
// This object is used for handling connections to the Philips Hue API.
WiFiClient hueApiClient;

// Instantiate an object named http of the HTTPClient class. 
// This object is used for making HTTP requests.
HTTPClient http;

// ************************************* WiFi connection ************************************************

void connectToWiFi() {
    WiFi.config(staticIP, gateway, subnet);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);

        Serial.println("Connecting to WiFi... ");
    }
        Serial.println("Connected to WiFi");
        Serial.println("IP address: " + WiFi.localIP().toString()  );
        Serial.println("WiFi status: " + String(WiFi.status()));
}

// ************************************* MQTT connection ************************************************

void connectToMQTT() {
    // connect to mqtt broker
    mqttClient.setServer(mqttServer, mqttPort);

    while (!mqttClient.connected()) {

        Serial.print("Connecting to MQTT broker... ");
        if (mqttClient.connect("ESP8266Client_2", mqttUser, mqttPassword)) {
            
            Serial.println("Connected to MQTT broker.");
        } 
        else {
            Serial.print("Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println("Retrying in 5 seconds...");
            delay(5000);    
        }
    }
}

// ************************************* philips hue API (HTTP) functionality ************************************************

bool isMotionDetectedFromJSON(String jsonResponse) {
  // Parse the JSON response
  DynamicJsonDocument doc(1024); // Adjust the size based on your JSON response size
  deserializeJson(doc, jsonResponse);

  // Check if "presence" key is present and set to true
  if (doc.containsKey("state") && doc["state"].containsKey("presence")) {
    return doc["state"]["presence"];
  } else {
    // If key is not present, assume no motion
    return false;
  }
}

bool isMotionDetected() {
  
  // Construct the URL for the Philips Hue API (replace with your specific sensor ID)
  String url = "http://" + String(hueBridgeIP) + "/api/" + String(hueUser) + "/sensors/2";

  http.begin(hueApiClient, url); // Use WiFiClient object as an argument

  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    String response = http.getString();
    //Serial.println("Response from Hue API:");
    //Serial.println(response);

    // Parse the JSON response to check if motion is detected
    return isMotionDetectedFromJSON(response);
  } else {
    Serial.print("Hue API request failed, response code: ");
    Serial.println(httpResponseCode);
    return false;
  }
// Close the connection
  http.end();
}

void setup() {
    
    Serial.begin(115200);

    // Connect to Wi-Fi
    connectToWiFi();

    // Connect to MQTT broker
    connectToMQTT();

    // Initialize pins for relays
    pinMode(relayPin1, OUTPUT);
    pinMode(relayPin2, OUTPUT);

    // Ensure relays are initially off
    digitalWrite(relayPin1, HIGH);
    digitalWrite(relayPin2, HIGH);    
}

void loop() {

mqttClient.loop();

if (isMotionDetected()) {
  Serial.println("Motion detected!");
  digitalWrite(relayPin1, LOW);
  StartTimer = true;
}

}







  

  


