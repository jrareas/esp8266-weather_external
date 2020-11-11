#include "battery.h"
#include "resources.h"
#ifdef ESP32
extern "C" int rom_phy_get_vdd33();
#endif

//#define CUSTOM_VCC

#ifdef CUSTOM_VCC
const int VCC_PIN = A0;
#endif

void Battery::begin(double battery_vcc_rate) {
  #ifdef CUSTOM_VCC
  pinMode(VCC_PIN, INPUT);
  #endif
  Battery::battery_vcc = battery_vcc_rate;
}

bool Battery::lowBattery(double volt) {
  return volt < Battery::battery_vcc * .91;
}

const unsigned char* Battery::getBatteryBmp(double curvolt) {
  //double curvolt = Battery::getCurVolt();
  //Serial.print("curv:");
  //Serial.print(curvolt);
  //Serial.print(",batvcc:");
  //Serial.println(battery_vcc);
  
  if(curvolt >= Battery::battery_vcc * .98) {
    return  battery_full;
  } else if (curvolt > (Battery::battery_vcc * .95)) {
    return battery_75;
  } else if(curvolt >= (Battery::battery_vcc * .91))
  {
    return battery_half;
  } 
  else if (curvolt > (Battery::battery_vcc * .88)) {
    return battery_33; 
  } else
  {
    return battery_empty;
  }
}

double Battery::getCurVolt() {
  
  #ifdef ESP8266
    #ifdef CUSTOM_VCC
    int Vvalue;
    for(unsigned int i=0;i<10;i++){
      Vvalue=Vvalue+analogRead(VCC_PIN);         //Read analog Voltage
      delay(5);                              //ADC stable
    }
    Vvalue = Vvalue/10;
    Serial.print("battery read:");
    Serial.println(Vvalue);
    return (battery_vcc*Vvalue)/1024;
    #else
      return ESP.getVcc() / 1000.00;
    #endif
  #elif defined ESP32
  return ((float)rom_phy_get_vdd33()) / 1000.00;
  #endif
}