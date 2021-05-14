//include libraries

#include <WiFi.h>
#include "Wire.h"
#include <Helios.h>
#include "time.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>

//initialize objects
Helios helios;

#define BNO055_SAMPLERATE_DELAY_MS (100)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//variables
const double latitude = 40.677160; //variable for latitude
const double longitude = -73.676260; //variable for longitude
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -0*3600;
const int   daylightOffset_sec = 0*3600;
double dAzimuth;
double dElevation;
double currentAzimuth;
double currentElevation;
double temp = 0;
double temp2 = 0;
double temp3 = 0;

//wifi
const char* ssid       = "nethear96";
const char* password   = "shinyunicorn404";
//functions

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void updateSolar() { //gets Time, prints time
  struct tm tm;
  if(!getLocalTime(&tm)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&tm, "%A, %B %d %Y %H:%M:%S");
  //int iYear, int iMonth, int iDay, double dHours, double dMinutes, double dSeconds, double dLongitude, double dLatitude); 
  Serial.print("Year: ");
  Serial.println((tm.tm_year-100));
  Serial.print("Month: ");
  Serial.println(tm.tm_mon+1);
  Serial.print("Day: ");
  Serial.println(tm.tm_mday);
  Serial.print("Hour: ");
  Serial.println(tm.tm_hour);
  Serial.print("Min: ");
  Serial.println(tm.tm_min);
  Serial.print("Sec: ");
  Serial.println(tm.tm_sec);
  Serial.print("Lat: ");
  Serial.println(latitude);
  Serial.print("Long: ");
  Serial.println(longitude); 
  
  helios.calcSunPos((tm.tm_year-100), (tm.tm_mon+1), tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, longitude, latitude); 
  dAzimuth=(helios.dAzimuth+10);
  dElevation=(helios.dElevation-10);
  Serial.print("Actual Azimuth = ");
  Serial.println(dAzimuth);
  Serial.print("Actual Elevation = ");
  Serial.println(dElevation);
}

void orientation(){
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> euler = quat.toEuler(); //find quaternion.h
  if (abs(degrees(euler.x()-temp)) > 1 or abs(degrees(euler.y()-temp2))) {
  temp = euler.x();
  temp3 = euler.z();
  currentAzimuth = 180-degrees(euler.x());
  currentElevation = 90-degrees(euler.z());
  Serial.print("Current Azimuth Angle: "); Serial.println(currentAzimuth);
  Serial.print("Current Elevation Angle: "); Serial.println(currentElevation);
  }
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
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
  //initialize accelerometer:
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Outside Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  updateSolar();
  for(int i = 0; i<10; i++) {
  orientation(); 
  }
  delay(20000);
}
