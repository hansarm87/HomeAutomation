// myCredentials.h
#ifndef myCredentials_h
#define myCredentials_h

#include <Arduino.h>

class myCredentials {
public:
  static const char* ssid;
  static const char* password;

  static const char* hueUser;

  static const char* mqttUser;
  static const char* mqttPassword;

  // Add other credentials or configuration parameters as needed
};

#endif
