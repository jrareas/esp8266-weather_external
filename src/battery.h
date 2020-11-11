#include <Arduino.h>

class Battery
{
public:
  void begin(double battery_vcc_rate);
  double getCurVolt();
  const unsigned char* getBatteryBmp(double curvolt);
  bool lowBattery(double volt);
private:
  double battery_vcc;
  
  void static wakeUp();
};