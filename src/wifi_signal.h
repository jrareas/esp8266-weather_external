#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif defined ESP32
#include <WiFi.h>
#endif

class WifiSignal{
public:

#ifdef ESP8266
  void begin(ESP8266WiFiClass wifi);
#elif defined ESP32
  void begin(WiFiClass wifi);
#endif
  double getSignalStrength();
  const unsigned char* getWifiBmp();
private:
#ifdef ESP8266
  ESP8266WiFiClass wifi;
#elif defined ESP32
  WiFiClass wifi;
#endif
};