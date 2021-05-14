//include libraries
#include "Wire.h"
#include <Helios.h>
#include "time.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MAX31865.h>
#include "Adafruit_MPRLS.h"
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
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28)
// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
//variables
const double latitude = 40.677160; //variable for latitude
const double longitude = -73.676260; //variable for longitude
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -4*3600;
const int   daylightOffset_sec = 0*3600;
double dAzimuth;
double dElevation;
//pins
const int valvePin = 23; //SERVO on schematic
const int AzStepperDir = 25; //AZ_Enable on Schematic
const int AzStepperStep = 26; //AZ_ on schematic
const int ElStepperDir = 19; //SERVO on schematic
const int ElStepperStep = 18; //SERVO on schematic
const int Pump_H2O = 4;
const int channel_H2O = 0;
const int frequency = 5000;
const int resolution = 8;

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
  dAzimuth=helios.dAzimuth;
  dElevation=helios.dElevation;
  Serial.println("Azimuth = " + dAzimuth + "; Elevation = " + dElevation);
}

void orientation(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.println("");
}

void pumpWater(){
  //if temp chamber > 100C + tolerance, then pump some water
  if(waterRTD > (100+5)) {
  //open solenoid valve
  
  ledcWrite(channel_H2O, 255);
}

void pressure() {
  float pressure_hPa = mpr.readPressure();
  Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
  Serial.print("Pressure (PSI): "); Serial.println(pressure_hPa / 68.947572932);
}

void setup() {
  //attach pins
  egRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  waterRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  pinMode(Pump_H2O, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(AzStepperDir, OUTPUT);
  pinMode(AzStepperStep, OUTPUT);
  pinMode(ElStepperDir, OUTPUT);
  pinMode(ElStepperStep, OUTPUT);
  ledcSetup(channeH2O,frequency,resolution);
  ledcAttachPin(Pump_H2O, channel_H2O);
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
  // put your setup code here, to run once:
  //define pressure sensor 1
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");

}

void loop() {
  // put your main code here, to run repeatedly:
  //if pressure sensor 1<2 {turn on motor 1}
  updateSolar();
  egRTD.temperature(RNOMINAL, RREF); //read ethylene glycol temp
  waterRTD.temperature(RNOMINAL, RREF); //read water temp
  orientation();
  
}
