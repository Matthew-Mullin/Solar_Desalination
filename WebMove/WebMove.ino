  /*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-web-server-AzStepperDirs-momentary-switch/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <WiFi.h>
#include "Adafruit_MPRLS.h"
#ifdef ESP32
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <Helios.h>
#include "time.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Helios helios;
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(15, 13, 12, 14);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(17);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

#define BNO055_SAMPLERATE_DELAY_MS (100);
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "nethear96_EXT";
const char* password = "shinyunicorn404";

const double latitude = 40.677160; //variable for latitude
const double longitude = -73.676260; //variable for longitude
const int stepperDelay = 3000;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -0*3600;
const int   daylightOffset_sec = 0*3600;
double dAzimuth;
double dElevation;
double currentAzimuth = 0;
double currentElevation = 0;
double temp1 = 0;
double temp2 = 0;
double temp3 = 0;

const int valvePin = 23; //SERVO on schematic
const int Pump_H2O = 4;
const int Pump_Salt = 33;

const int AzEnable = 27;
const int AzStepperDir = 25; //AZ_Enable on Schematic LEFT AZ PIN
const int AzStepperStep = 26; //AZ_1 on schematic MIDDLE AZ PIN
const int ElStepperDir = 19; //EL_Enable on schematic RIGHT EL PIN
const int ElStepperStep = 18; //El_1 on schematic MIDDLE EL PIN
const int ElEnable = 5;
const int stepsPerRevolution = 19910;
int AzUpState = 0;
int AzDownState = 0;  
int ElUpState = 0;
int ElDownState = 0;

int AutoModeState = 0;
int CalibrateState = 0;

AsyncWebServer server(80);

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

String updateSolar() { //gets Time, prints time
  struct tm tm;
  if(!getLocalTime(&tm)){
    Serial.println("Failed to obtain time");
    return "Can't get time";
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
  return("Solar Azimuth: " + String(dAzimuth) + "; Solar Elevation: " + String(dElevation));
}

void pumpWater(){
  //if temp chamber > 100C + tolerance, then pump some water
  double temp = thermo.temperature(RNOMINAL, RREF);
  Serial.print("Water Temp: ");
  Serial.print(temp);
  Serial.println(" C");
  if(temp > (100+5)) {
  //open solenoid valve
  digitalWrite(valvePin, HIGH);
  delay(1000);
  digitalWrite(Pump_H2O, HIGH);
  delay(10000);
  digitalWrite(Pump_Salt, HIGH);
  delay(10000);
  }
}

void stepMotor(int dirPin, int stepPin, double numDegrees, int amountDelay) {
  int steps = (stepsPerRevolution/360)*numDegrees;
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
  digitalWrite(ElEnable, HIGH);
  digitalWrite(AzEnable, HIGH);
  stepMotor(ElStepperDir, ElStepperStep, elevation, stepperDelay);
  stepMotor(AzStepperDir, AzStepperStep, azimuth, stepperDelay);
  digitalWrite(ElEnable, LOW);
  digitalWrite(AzEnable, LOW);
}

String accelTemp() {
  int8_t accelTemp = bno.getTemp();
  Serial.print("Accel Temp: ");
  Serial.print(accelTemp);
  Serial.println(" C");
  Serial.println("");
  return String(accelTemp);
}

String readWaterTemp() {
  uint16_t rtd = thermo.readRTD();
  
  float ratio = rtd;
  ratio /= 32768;
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Water Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));
  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
    return "--";
  } else {
    return String(thermo.temperature(RNOMINAL, RREF));
  }
  Serial.println();
}

String readPress() {
  float PressureHPa = mpr.readPressure();
  //Serial.print("Pressure (hPa): "); Serial.println(PressureHPa);
  Serial.print("Pressure (PSI): "); Serial.println(PressureHPa / 68.947572932);
  return String(PressureHPa / 68.947572932);
}

String orientation(){
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> euler = quat.toEuler(); //find quaternion.h
  if (abs(degrees(euler.x()-temp1)) > 1 or abs(degrees(euler.y()-temp2))) {
  temp1 = euler.x();
  temp3 = euler.z();
  currentAzimuth = 270-degrees(euler.x());
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
  return("Current Azimuth: " + String(currentAzimuth) + "; Current Elevation: " + String(currentElevation));
}

void AutoMode() {
  Serial.println("Automode Engaged");
  updateSolar();
  orientation();
  moveHeliostat();
}

// HTML web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <title>ESP Pushbutton Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
      .button {
        padding: 10px 20px;
        font-size: 24px;
        text-align: center;
        outline: none;
        color: #fff;
        background-color: #2f4468;
        border: none;
        border-radius: 5px;
        box-shadow: 0 6px #999;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }  
      .button:hover {background-color: #1f2e45}
      .button:active {
        background-color: #1f2e45;
        box-shadow: 0 4px #666;
        transform: translateY(2px);
      }
    </style>
  </head>
  <body>
    <h1>ESP Pushbutton Web Server</h1>
    <button class="button" onmousedown="toggleCheckbox('AzUpOn');" ontouchstart="toggleCheckbox('AzUpOn');" onmouseup="toggleCheckbox('AzUpOff');" ontouchend="toggleCheckbox('AzUpOff');">AzLeft</button>
    <button class="button" onmousedown="toggleCheckbox('AzDownOn');" ontouchstart="toggleCheckbox('AzDownOn');" onmouseup="toggleCheckbox('AzDownOff');" ontouchend="toggleCheckbox('AzDownOff');">AzRight</button>
    <button class="button" onmousedown="toggleCheckbox('ElUpOn');" ontouchstart="toggleCheckbox('ElUpOn');" onmouseup="toggleCheckbox('ElUpOff');" ontouchend="toggleCheckbox('ElUpOff');">ElUp</button>
    <button class="button" onmousedown="toggleCheckbox('ElDownOn');" ontouchstart="toggleCheckbox('ElDownOn');" onmouseup="toggleCheckbox('ElDownOff');" ontouchend="toggleCheckbox('ElDownOff');">ElDown</button>
    <button class="button" onmousedown="toggleCheckbox('PumpH2OOn');" ontouchstart="toggleCheckbox('PumpH2OOn');" onmouseup="toggleCheckbox('PumpH2OOff');" ontouchend="toggleCheckbox('PumpH2OOff');">Pump H2O</button>
    <button class="button" onmousedown="toggleCheckbox('ValveOn');" ontouchstart="toggleCheckbox('ValveOn');" onmouseup="toggleCheckbox('ValveOff');" ontouchend="toggleCheckbox('ValveOff');">Valve</button>
    <button class="button" onmousedown="toggleCheckbox('PumpSaltOn');" ontouchstart="toggleCheckbox('PumpSaltOn');" onmouseup="toggleCheckbox('PumpSaltOff');" ontouchend="toggleCheckbox('PumpSaltOff');">Pump Salt</button>
    <button class="button" onmousedown="toggleCheckbox('AutoModeOn');" ontouchstart="toggleCheckbox('AutoModeOn');">AutoMode On</button>
    <button class="button" onmousedown="toggleCheckbox('AutoModeOff');" ontouchstart="toggleCheckbox('AutoModeOff');">AutoMode Off</button>
    <button class="button" onmousedown="toggleCheckbox('CalibrateOn');" ontouchstart="toggleCheckbox('CalibrateOn');">Calibrate On</button>
    <button class="button" onmousedown="toggleCheckbox('CalibrateOff');" ontouchstart="toggleCheckbox('CalibrateOff');">Calibrate Off</button>
    <h2>Sensor Data</h2>
    <p>
      <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
      <span class="dht-labels">Water Temperature</span> 
      <span id="watertemperature">%WATERTEMPERATURE%</span>
      <sup class="units">&deg;C</sup>
    </p> 
    <p>
    <i class="fas fa-tint" style="color:#00add6;"></i> 
    <span class="dht-labels">Pressure</span>
    <span id="pressure">%PRESSURE%</span>
    <span class="units">psi</span>
  </p>
  <p>
    <i class="fas fa-tint" style="color:#00add6;"></i> 
    <span class="dht-labels">Orientation</span>
    <span id="orientation">%ORIENTATION%</span>
  </p>
  <p>
    <i class="fas fa-tint" style="color:#00add6;"></i> 
    <span class="dht-labels">Accelerometer Temp</span>
    <span id="acceltemp">%ACCELTEMP%</span>
    <sup class="units">&deg;C</sup>
  </p>
  </body>   
  <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/" + x, true);
     xhr.send();
   }
   
   setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("watertemperature").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/watertemperature", true);
  xhttp.send();
}, 10000 ) ;

  setInterval(function ( ) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("pressure").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "/pressure", true);
    xhttp.send();
  }, 10000 ) ;

  setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("orientation").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/orientation", true);
  xhttp.send();
}, 10000 ) ;

  setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("acceltemp").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/acceltemp", true);
  xhttp.send();
}, 10000 ) ;

 /* setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("solarangle").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/solarangle", true);
  xhttp.send();
}, 10000 ) ;*/

  </script>
  </body>
  </html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// Replaces placeholder with DHT values
String processor(const String& var){
  //Serial.println(var);
  if(var == "WATERTEMPERATURE"){
    return readWaterTemp();
  }
  else if(var == "PRESSURE"){
    return readPress();
  }
  else if(var == "ORIENTATION"){
    return orientation();
  }
  else if(var == "ACCELTEMP"){
    return accelTemp();
  }
/*  else if(var == "SOLARANGLE"){
    return updateSolar();
  }*/
  return String();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  //define pressure sensor 1
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");

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
  Serial.print("Accel Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(true);
  // put your setup code here, to run once:
  
  pinMode(AzStepperDir, OUTPUT);
  digitalWrite(AzStepperDir, LOW);
  pinMode(AzStepperStep, OUTPUT);
  digitalWrite(AzStepperStep, LOW);
  pinMode(ElStepperDir, OUTPUT);
  digitalWrite(ElStepperDir, LOW);
  pinMode(ElStepperStep, OUTPUT);
  digitalWrite(ElStepperStep, LOW);
  pinMode(ElEnable, OUTPUT);
  digitalWrite(ElEnable, HIGH);
  pinMode(AzEnable, OUTPUT);
  digitalWrite(AzEnable, HIGH);
  pinMode(Pump_H2O, OUTPUT);
  pinMode(Pump_Salt, OUTPUT);
  pinMode(valvePin, OUTPUT);

  
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.on("/watertemperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readWaterTemp().c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readPress().c_str());
  });
  server.on("/orientation", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", orientation().c_str());
  });
  server.on("/acceltemp", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", accelTemp().c_str());
  });
/*  server.on("/solarangle", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", updateSolar().c_str());
  });*/

  // Receive an HTTP GET request
  server.on("/AzUpOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Az Up On");
    AzUpState = HIGH;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/AzUpOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Az Up Off");
    AzUpState = LOW;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/AzDownOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Az Down On");
    AzDownState = HIGH;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/AzDownOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Az Down Off");
    AzDownState = LOW;
    request->send(200, "text/plain", "ok");
  });

    // Receive an HTTP GET request
  server.on("/ElUpOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("El Up On");
    ElUpState = HIGH;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/ElUpOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("El Up Off");
    ElUpState = LOW;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/ElDownOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("El Down On");
    ElDownState = HIGH;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/ElDownOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("El Down Off");
    ElDownState = LOW;
    request->send(200, "text/plain", "ok");
  });
  
  // Receive an HTTP GET request
  server.on("/PumpH2OOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Pump H2O On");
    digitalWrite(Pump_H2O, HIGH);
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/PumpH2OOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Pump H2O Off");
    digitalWrite(Pump_H2O, LOW);
    request->send(200, "text/plain", "ok");
  });
  
  // Receive an HTTP GET request
  server.on("/ValveOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Pump Valve On");
    digitalWrite(valvePin, HIGH);
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/ValveOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Valve Off");
    digitalWrite(valvePin, LOW);
    request->send(200, "text/plain", "ok");
  });  

  // Receive an HTTP GET request
  server.on("/PumpSaltOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Pump Salt On");
    digitalWrite(Pump_Salt, HIGH);
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/PumpSaltOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Pump Salt Off");
    digitalWrite(Pump_Salt, LOW);
    request->send(200, "text/plain", "ok");
  });

    // Receive an HTTP GET request
  server.on("/AutoModeOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AutoModeState = 1;
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/AutoModeOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AutoModeState = 0;
    request->send(200, "text/plain", "ok");
  });

      // Receive an HTTP GET request
  server.on("/CalibrateOn", HTTP_GET, [] (AsyncWebServerRequest *request) {
    CalibrateState = 1;
    Serial.println("Calibrate On");
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/CalibrateOff", HTTP_GET, [] (AsyncWebServerRequest *request) {
    CalibrateState = 0;
    Serial.println("Calibration Complete");
    request->send(200, "text/plain", "ok");
  });
  
  server.onNotFound(notFound);
  server.begin();
}

void loop() {
    if(AzDownState == HIGH) {
    digitalWrite(AzEnable, LOW);
    Serial.println("Az Up");
    digitalWrite(AzStepperDir, HIGH);
    digitalWrite(AzStepperStep, HIGH);
    delayMicroseconds(stepperDelay);
    digitalWrite(AzStepperStep, LOW);
    delayMicroseconds(stepperDelay);
  }
  else if (AzUpState == HIGH) {
    digitalWrite(AzEnable, LOW);
    Serial.println("Az Down");
    digitalWrite(AzStepperDir, LOW);
    digitalWrite(AzStepperStep, HIGH);
    delayMicroseconds(stepperDelay);
    digitalWrite(AzStepperStep, LOW);
    delayMicroseconds(stepperDelay);    
  }
  else if (ElUpState == HIGH) {
    digitalWrite(ElEnable, LOW);
    Serial.println("El Up");
    digitalWrite(ElStepperDir, HIGH);
    digitalWrite(ElStepperStep, HIGH);
    delayMicroseconds(stepperDelay);
    digitalWrite(ElStepperStep, LOW);
    delayMicroseconds(stepperDelay);   
  }
  else if (ElDownState == HIGH) {
    digitalWrite(ElEnable, LOW);
    Serial.println("El Down");
    digitalWrite(ElStepperDir, LOW);
    digitalWrite(ElStepperStep, HIGH);
    delayMicroseconds(stepperDelay);
    digitalWrite(ElStepperStep, LOW);
    delayMicroseconds(stepperDelay);   
  }
  else if(ElDownState == LOW && ElUpState == LOW && AzDownState == LOW && AzUpState == LOW) {
    digitalWrite(ElEnable, HIGH);
    digitalWrite(AzEnable, HIGH);
    digitalWrite(AzStepperDir, LOW);
    digitalWrite(AzStepperStep, LOW);
    digitalWrite(ElStepperDir, LOW);
    digitalWrite(ElStepperStep, LOW);
    vTaskDelay(10);
  }
  if(AutoModeState == 1) {
    AutoMode();
    delay(3000);
  }
  if(CalibrateState == 1) {
    digitalWrite(ElEnable, HIGH);
    digitalWrite(AzEnable, HIGH);
    orientation();
    delay(1500);
  }
}
