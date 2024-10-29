// myCredentials.h
#ifndef myCredentials_h
#define myCredentials_h

#include <Arduino.h>

class myCredentials {
public:
    static const char* ssid;
    static const char* password;
    static const char* mqttUser;
    static const char* mqttPassword;
};

#endif