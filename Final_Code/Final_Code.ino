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
#define RREF      4300.0;
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100);
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1;  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1;  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS pitotPressure = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
//variables
const double latitude = 40.677160; //variable for latitude
const double longitude = -73.676260; //variable for longitude
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -0*3600;
const int   daylightOffset_sec = 0*3600;
double dAzimuth;
double dElevation;
double currentAzimuth;
double currentdElevation;
//pins
const int valve = 23; //SERVO on schematic
const int AzStepperDir = 25; //AZ_Enable on Schematic
const int AzStepperStep = 26; //AZ_ on schematic
const int ElStepperDir = 19; //SERVO on schematic
const int ElStepperStep = 18; //SERVO on schematic
const int stepsPerRevolution = 19910;
const int Pump_H2O = 4;

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

void pumpWater(){
  //if temp chamber > 100C + tolerance, then pump some water
  double temp = waterRTD.temperature(RNOMINAL, RREF);
  Serial.print("Water Temp: ");
  Serial.print(temp);
  Serial.println(" C");
  if(temp > (100+5)) {
  //open solenoid valve
  digitalWrite(valve, HIGH);
  delay(1000);
  digitalWrite(Pump_H2O, HIGH);
  delay(5000);
  }
}

void recordPressure() {
  float ambientPressure = 3000000; //in Pa
  float pitotPressureHPa = pitotPressure.readPressure();
  Serial.print("Pitot Pressure (hPa): "); Serial.println(pitotPressureHPa);
  Serial.print("Pitot Pressure (PSI): "); Serial.println(pitotPressureHPa / 68.947572932);
  float windSpeed = sqrt(2*((10*pitotPressureHPa)-(10*ambientPressureHPa))) //windSpeed in m/s
  Serial.println("Wind speed: " + windSpeed + " m/s");
}

void stepMotor(int dirPin, int stepPin, double numDegrees, int amountDelay) {
  int steps = (stepsPerRevolution/360)*numDegrees
  digitalWrite(dirPin, HIGH);
  for (int x = 0; x<steps; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(amountDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(amountDelay);
  }
}

void moveHeliostat() {
  double elevation = dElevation-currentElevation;
  double azimuth = dAzimuth-currentAzimuth;
  stepMotor(ElStepperDir, ElStepperStep, elevation, 1000);
  stepMotor(AzStepperDir, AzStepperStep, azimuth, 1000);
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
  /*for(int i = 0; i<3; i++) {
  recordPressure();
  pumpWater();
  delay(1000*60);
  }*/
  for(int i = 0; i<3; i++) {
  updateSolar();
  orientation();
  moveHeliostat();  
  delay(1000*3);
  }
}
