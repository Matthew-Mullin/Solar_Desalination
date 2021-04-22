#include "SPI.h"
#include <Adafruit_MAX31865.h> // Adafruit's Header file 

// https://unininu.tistory.com/219

// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 egRTD = Adafruit_MAX31865(15, 13, 12, 14);
// Adafruit_MAX31865 waterRTD = Adafruit_MAX31865(17, 13, 12, 14);

// use hardware SPI, Write your CS pin
Adafruit_MAX31865 egRTD = Adafruit_MAX31865(15);
Adafruit_MAX31865 waterRTD = Adafruit_MAX31865(17); //What's the CS pin for another temp. sensor???

// #define RREF      4300.0 // 4.3Kohm 
// #define RNOMINAL  1000.0 // PT1000

float operatMax31865_1(void) {
  float temperature_1 = egRTD.temperature(RNOMINAL, RREF);
  return temperature_1;
}

float operatMax31865_2(void) {
  float temperature_2 = waterRTD.temperature(RNOMINAL, RREF);
  return temperature_2;
}

void setup() {
  // put your setup code here, to run once:
 egRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
 waterRTD.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("PT1000_1: "); Serial.print(operatMax31865_1()); Serial.print("/ ");
  Serial.print("PT1000_2: "); Serial.print(operatMax31865_2()); Serial.print("/ ");
}
