/*
 * main.cpp
 *
 *  Created on: Nov. 8, 2020
 *      Author: jrareas
 */




#include <Arduino.h>
#include "thingspeak.h"
#ifdef ESP8266
#include <ESP8266WiFi.h>
ADC_MODE(ADC_VCC);
#elif defined ESP32
#include <WiFi.h>
#endif
#include <WiFiManager.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "resources.h"
#include "secrets.h"



//#define CUSTOM_DEEP_SLEEP
#define ets_wdt_disable ((void (*)(void))0x400030f0)
#define ets_delay_us ((void (*)(int))0x40002ecc)

#define _R (uint32_t *)0x60000700
void nk_deep_sleep(uint64_t time)
{
  ets_wdt_disable();
  *(_R + 4) = 0;
  *(_R + 17) = 4;
  *(_R + 1) = *(_R + 7) + 5;
  *(_R + 6) = 8;
  *(_R + 2) = 1 << 20;
  ets_delay_us(10);
  *(_R + 39) = 0x11;
  *(_R + 40) = 3;
  *(_R) &= 0xFCF;
  *(_R + 1) = *(_R + 7) + (45*(time >> 8));
  *(_R + 16) = 0x7F;
  *(_R + 2) = 1 << 20;
  __asm volatile ("waiti 0");
}


//#define OLED
#define MY_SERIAL
#define THINGSPEAK

#ifdef THINGSPEAK
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#endif


#ifdef OLED
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4);
#endif

#include "battery.h"
#include "wifi_signal.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

const byte interruptPin = 13;


//const char *ssid = "weather_station";
//const char *password = "anothaone";
const int analogInPin = 0;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the potentiometer
int outputValue = 0;        // value sent to server


float temperature, humidity, pressure, altitude;
double curvolt, wifilevel;

Battery battery;
WifiSignal wifisignal;

void sendData();
String getUrl();
void readData();

void setup() {
  #ifdef MY_SERIAL
  Serial.begin(115200);
  Serial.setTimeout(2000);
  //if (ESP.getResetReason() != "External System") {
  //  delay(1000); // wait 1 sec in case
  //}
  // Wait for serial to initialize.
  while(!Serial) { }

  //Serial.println(ESP.getResetReason());
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");

  Serial.println(WiFi.softAP("ESPsoftAP_01") ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");

  Serial.println(WiFi.softAPIP());
  #endif
  #ifdef OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tn);
  u8g2.setFontMode(0);
  #endif

//  Serial.println("Going into deep sleep for 20 seconds");
//  ESP.deepSleep(0); // 20e6 is 20 microseconds

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(60);
  //wifiManager.resetSettings();
  if(!wifiManager.autoConnect("AutoConnectAP")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
#ifdef CUSTOM_DEEP_SLEEP
	  nk_deep_sleep(180e6);
#else
	  ESP.deepSleep(180e6);
#endif
  }
  wifiManager.autoConnect("AutoConnectAP");

  wifisignal.begin(WiFi);
  battery.begin(4.2);
  Serial.println("Open BME");
  bme.begin(0x76);
  Serial.println("Open BME Done");
  sendData();
  #ifdef MY_SERIAL
  Serial.println("Going into deep sleep now");
  #endif
  #ifdef CUSTOM_DEEP_SLEEP
  nk_deep_sleep(180e6);
  #else
  ESP.deepSleep(180e6);
  #endif
}

void sendData() {
  readData();
  sensorValue = analogRead(A0);
  // map to range. The pot goes from about 3 to 1023. This makes the sent value be between 0 and 999 to fit on the OLED
  outputValue = map(sensorValue, 3, 1023, 0, 999);

	#ifdef THINGSPEAK
  	  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  	  client->setInsecure();
  	  HTTPClient https;
  	  const char * host = "https://api.thingspeak.com";
  	  if (https.begin(*client, String(host) + getUrl())) {
  		int httpCode = https.GET();

  		    // httpCode will be negative on error
  		    if (httpCode > 0) {
  		      // HTTP header has been send and Server response header has been handled
  		      Serial.printf("[HTTPS] GET... code: %d\n", httpCode);
  		      // file found at server?
  		      if (httpCode == HTTP_CODE_OK) {
  		        String payload = https.getString();
  		      }
  		    } else {
  		      Serial.printf("[HTTPS] GET... failed, error: %s\n\r", https.errorToString(httpCode).c_str());
  		    }

  		    https.end();
  	  }
	#else
  	  WiFiClient client;
  	  const char * host = "192.168.4.1";
  	  const int httpPort = 80;
  	  if (!client.connect(host, httpPort)) {
  	    Serial.println("connection failed");
  	    return;
  	  }
  	// This will send the request to the server
  	  client.print(String("GET ") + getUrl() + " HTTP/1.1\r\n" +
  	               "Host: " + host + "\r\n" +
  	               "Connection: close\r\n\r\n");
  	  unsigned long timeout = millis();
  	  while (client.available() == 0) {
  	    if (millis() - timeout > 5000) {
  	      Serial.println(">>> Client Timeout !");
  	      client.stop();
  	      return;
  	    }
  	  }
	#endif


  #ifdef OLED
  u8g2.firstPage();
  char buff[5];
  dtostrf(temperature, 2, 2, buff);
  u8g2.drawUTF8(0, 30, buff);
  dtostrf(pressure, 2, 2, buff);
  u8g2.drawUTF8(40, 30, buff);

  dtostrf(battery.getCurVolt(), 2, 2, buff);
  u8g2.drawBitmap(110, 2, 2, 8, battery.getBatteryBmp());
  u8g2.drawBitmap(0, 0, 1, 8, wifisignal.getWifiBmp());

  u8g2.nextPage();

  u8g2.nextPage();
  #endif
}
String getUrl() {
#ifdef THINGSPEAK
	String url = "/update?api_key=";
	url += TS_WRITE_APIKEY;
#else
	String url = "/data";
#endif
  char buff[10];
  dtostrf(temperature, 2, 2, buff);
#ifdef THINGSPEAK
  url += "&field1=";
#else
  url += "?temp=";
#endif
  url += buff;
  dtostrf(pressure, 2, 2, buff);
#ifdef THINGSPEAK
  url += "&field2=";
#else
  url += "&press=";
#endif
  url += buff;
  curvolt = battery.getCurVolt();

  dtostrf(curvolt, 2, 2, buff);
  // We now create a URI for the request. Something like /data/?sensor_reading=123
#ifdef THINGSPEAK
  url += "&field3=";
#else
  url += "&batt=";
#endif
  url += buff;

  dtostrf(altitude, 2, 2, buff);
#ifdef THINGSPEAK
  url += "&field4=";
#else
  url += "&alt=";
#endif
  url += buff;


  dtostrf(humidity, 2, 2, buff);
#ifdef THINGSPEAK
  url += "&field5=";
#else
  url += "&hum=";
#endif
  url += buff;

  wifilevel = wifisignal.getSignalStrength();
  dtostrf(wifilevel, 2, 2, buff);
#ifdef THINGSPEAK
  url += "&field6=";
#else
  url += "&wifi=";
#endif
  url += buff;
  #ifdef MY_SERIAL
  Serial.print("Url:");
  Serial.println(url);
  #endif
  return url;
}

void readData() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  #ifdef MY_SERIAL
  Serial.print("Temp:");
  Serial.println(temperature);
  Serial.print("Hum:");
  Serial.println(humidity);
  Serial.print("Press:");
  Serial.println(pressure);
  Serial.print("Altitude:");
  Serial.println(altitude);
  #endif
}

void loop() {

}
