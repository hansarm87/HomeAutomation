#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

const char* ssid = "IPUP";
const char* password = "HansOgPeter2300";

IPAddress staticIP(192,168,0,22);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);

//Philips hue API details
const char* hueBridgeIP = "192.168.0.2";
const char* hueUsername = "3dOuJDjKS0i1NiAyIBK0T1Zt0VYPXGQ3sE-o1iqb";

// MQTT broker details
const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;
const char* mqttUsername = "hansa";
const char* mqttPassword = "Hkglape8266";

// pinout for relay
const int relayPin1 = D1;
const int relayPin2 = D2;

// Function prototypes
bool isMotionDetected();
bool isMotionDetectedFromJSON(String jsonResponse);

// Timer variables
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const int onDelayTime = 5 * 60 * 1000; // 5 minutes in milliseconds

bool relayOn = false; // To track the state of the relay

// Initialize the Wi-Fi client and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// ************************************* MQTT functionality ************************************************

void callBack(String topic, byte* message, unsigned int length) {
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();


if (topic == "kitchen_hallway/relay1/livingroomLamp") {
  Serial.print("changing livingroomLamp to ");
  if (messageTemp == "on") {
    // The relay is triggered when the input goes below about 2 V. 
    // That means if you send a LOW signal from the Arduino, the relay turns on,
    digitalWrite(relayPin2, LOW); 
    digitalWrite(LED_BUILTIN, LOW); // NC configured.. LOW signal activates LED 
    //Serial.print("on");
    Serial.printf( "%s\n", messageTemp);
    
  }
  else if (messageTemp == "off") {
    // Because we’re using a normally open configuration, there is no contact    
    // between the COM and NO sockets unless you trigger the relay. 
    digitalWrite(relayPin2, HIGH); 
    digitalWrite(LED_BUILTIN, HIGH); // NC configured.. HIGH signal deactivates LED  
    //Serial.print("off");
    Serial.printf( "%s\n", messageTemp);
  }
  Serial.println();
  }
}

void reconnect() {
  // Loop until connected to MQTT broker
  while (!client.connected()) {
    Serial.println("Reconnecting to MQTT broker...");
    if (client.connect("ESP8266Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

//************************* SETUP ***************************************************************************

void setup() {
  Serial.begin(115200);
 
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);

  // Ensure relays are initially off
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);

  //to keep track relay1 state
  relayOn = false;

  WiFi.config(staticIP, gateway, subnet);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Set MQTT server and callback function
    client.setServer(mqttServer, mqttPort);
    //client.setCallback(callBack);
    client.setCallback(callBack);
    // Connect to MQTT broker
    while (!client.connected()) {
        Serial.println("Connecting to MQTT broker...");
        if (client.connect("ESP8266Client", mqttUsername, mqttPassword)) {
        Serial.println("Connected to MQTT broker");
        client.subscribe("kitchen_hallway/relay1/livingroomLamp");
        } else {
        Serial.print("Failed, rc=");
        Serial.print(client.state());
        Serial.println(" Retrying in 5 seconds...");
        delay(5000);
        }
    }
}

//************************* LOOP *****************************************************************************

void loop() {

  // Handle MQTT client events
  client.loop();

  // Check if motion is detected
  if (isMotionDetected()) {
    // If motion is detected and relay is not already on, turn on the relay
    if (!relayOn) {
      digitalWrite(relayPin1, LOW); // Note the change to LOW for normally open relay
      relayOn = true;
      previousTime = millis(); // Reset the timer
    }
  } else {
    // If no motion, turn off the relay after 5 minutes
    if (relayOn && (currentTime - previousTime >= onDelayTime)) {
      digitalWrite(relayPin1, HIGH); // Note the change to HIGH for normally open relay
      relayOn = false;
    }
  }

  // Update the current time
  currentTime = millis();

  // Reconnect if MQTT connection is lost
  if (!client.connected()) {
    reconnect();
  }

  

  delay(100); // Adjust delay as needed
}

// ************************************* philips hue API (HTTP) functionality ************************************************

bool isMotionDetected() {
  // Use HTTPClient with WiFiClient object to send a GET request to the Philips Hue API
  WiFiClient client;
  HTTPClient http;

  // Construct the URL for the Philips Hue API (replace with your specific sensor ID)
  String url = "http://" + String(hueBridgeIP) + "/api/" + String(hueUsername) + "/sensors/2";

  http.begin(client, url); // Use WiFiClient object as an argument

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


