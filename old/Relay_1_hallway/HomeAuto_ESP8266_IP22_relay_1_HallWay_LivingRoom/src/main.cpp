#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Wi-Fi credentials
const char* ssid = "IPUP";
const char* password = "HansOgPeter2300";

IPAddress staticIP(192,168,0,22);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);



// MQTT broker details
const char* mqttServer = "192.168.0.246";
const int mqttPort = 1883;
const char* mqttUsername = "hansa";
const char* mqttPassword = "Hkglape8266";
//const char* mqttTopic = "";

// variables for pin connection
const int relayPin1 = D1; //gpio 5
const int relayPin2 = D2; //gpio 5

//timer variables


// Initialize the Wi-Fi client and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

void callBack(String topic, byte* message, unsigned int length){
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();


if (topic == "kitchen_hallway/relay1/hallwayLamp") {
  Serial.print("changing hallway lamp to ");
  if (messageTemp == "on") {
    // The relay is triggered when the input goes below about 2 V. 
    // That means if you send a LOW signal from the Arduino, the relay turns on,
    digitalWrite(relayPin1, LOW); 
    digitalWrite(LED_BUILTIN, LOW); // NC configured.. LOW signal activates LED 
    //Serial.print("on");
    Serial.printf( "%s\n", messageTemp);
    
  }
  else if (messageTemp == "off") {
    // Because we’re using a normally open configuration, there is no contact    
    // between the COM and NO sockets unless you trigger the relay. 
    digitalWrite(relayPin1, HIGH); 
    digitalWrite(LED_BUILTIN, HIGH); // NC configured.. HIGH signal deactivates LED  
    //Serial.print("off");
    Serial.printf( "%s\n", messageTemp);
  }
  Serial.println();
  
}
if (topic == "kitchen_hallway/relay2/livingroomLamp") {
  Serial.print("changing livingroom lamp to ");
  if (messageTemp == "on") {
    digitalWrite(relayPin2, LOW); // NO configured.. LOW signal activates relay
    digitalWrite(LED_BUILTIN, LOW);  
    //Serial.print("on");
    Serial.printf( "%s\n", messageTemp);
    
  }
  else if (messageTemp == "off") {
    digitalWrite(relayPin2, HIGH); // NO configured.. HIGH signal deactivates relay
    digitalWrite(LED_BUILTIN, HIGH);   
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

void setup() {
    Serial.begin(115200);

    pinMode(relayPin1, OUTPUT);
    pinMode(relayPin2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    WiFi.config(staticIP, gateway, subnet);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.printf("Connecting to %s\n", ssid);
    }
    Serial.println("Connected to Wi-Fi");
    Serial.print(" IP adresse:  ");
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
        client.subscribe("kitchen_hallway/relay1/hallwayLamp");
        client.subscribe("kitchen_hallway/relay2/livingroomLamp");
        } else {
        Serial.print("Failed, rc=");
        Serial.print(client.state());
        Serial.println(" Retrying in 5 seconds...");
        delay(5000);
        }
    }
}

void loop() {

    // Reconnect if MQTT connection is lost
  if (!client.connected()) {
    reconnect();
  }


  // create a string to hold data you want to publish (typically sensor data)
  // char dataString[10];
  // dtostrf(data, 4, 2, dataString);

  // Publish to MQTT topic:
  // mqttClient.publish(mqttTopic, dataString);

  // Handle MQTT client events
  client.loop();
  
}

