//include libraries
#include "Wire.h"
#include <Helios.h>
#include "time.h"
#include <Adafruit_MAX31865.h>
//initialize objects
Helios helios;
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 egRTD = Adafruit_MAX31865(15, 13, 12, 14);
Adafruit_MAX31865 waterRTD = Adafruit_MAX31865(17, 13, 12, 14);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0
//variables
const double latitude = 40.677160; //variable for latitude
const double longitude = -73.676260; //variable for longitude
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -4*3600;
const int   daylightOffset_sec = 0*3600;
double dAzimuth;
double dElevation;
//wifi
const char* ssid       = "nethear96";
const char* password   = "shinyunicorn404";
//functions

void updateSolar() {
  struct tm tm;
  if(!getLocalTime(&tm)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&tm, "%A, %B %d %Y %H:%M:%S");
  //int iYear, int iMonth, int iDay, double dHours, double dMinutes, double dSeconds, double dLongitude, double dLatitude); 
  helios.calcSunPos(tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, latitude, longitude); 
  dAzimuth=helios.dAzimuth;show("dAzimuth",dAzimuth,true);
  dElevation=helios.dElevation;show("dElevation",dElevation,true);
  Serial.println("Azimuth = " + dAzimuth + "; Elevation = " + dElevation);
}

void setup() {
  Serial.begin(115200); //initializing serial communication
  //connect to WiFi for time
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // put your setup code here, to run once:
  //define pressure sensor 1
  //define pressure sensor 2
  //define temperature sensor 1
  //define temperature sensor 2
  egRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  waterRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  //initialize servo pin
  //initialize stepper pins
  //initialize pump pins
}

void loop() {
  // put your main code here, to run repeatedly:
  //if pressure sensor 1<2 {turn on motor 1}
  updateSolar();
  egRTD.temperature(RNOMINAL, RREF); //read ethylene glycol temp
  waterRTD.temperature(RNOMINAL, RREF); //read water temp
}
