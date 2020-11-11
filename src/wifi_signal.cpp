#include "wifi_signal.h"
#include "resources.h"

const int RSSI_MAX =-50;// define maximum strength of signal in dBm
const int RSSI_MIN =-100;// define minimum strength of signal in dBm

#ifdef ESP8266
void WifiSignal::begin(ESP8266WiFiClass wifi) {
  WifiSignal::wifi = wifi;
}
#elif defined ESP32
void WifiSignal::begin(WiFiClass wifi) {
  WifiSignal::wifi = wifi;
}
#endif

const unsigned char* WifiSignal::getWifiBmp(){
  int quality = WifiSignal::getSignalStrength();
  if (quality > 85 ) {
    return wifi_full;
  } else if(quality > 65) {
    return wifi_75;
  } else if (quality > 45) {
    return wifi_half;
  } else if (quality > 25) {
    return wifi_33;
  } else {
    return wifi_none;
  }
}

double WifiSignal::getSignalStrength() {
  int dBm = WifiSignal::wifi.RSSI();
  int quality;
  if(dBm <= RSSI_MIN)
  {
      quality = 0;
  }
  else if(dBm >= RSSI_MAX)
  {  
      quality = 100;
  }
  else
  {
      quality = 2 * (dBm + 100);
 }
  Serial.print("Wifi Strength:");
  Serial.println(quality);
  return quality;
}