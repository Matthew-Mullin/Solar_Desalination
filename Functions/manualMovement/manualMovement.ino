const int AzStepperDir = 25; //AZ_Enable on Schematic
const int AzStepperStep = 26; //AZ_ on schematic
const int ElStepperDir = 19; //SERVO on schematic
const int ElStepperStep = 18; //SERVO on schematic
const int stepsPerRevolution = 19910;
const int AzUp = 17;
const int AzDown = 16;
const int ElUp = 23;
const int ElDown = 22;
int AzUpState = 0;
int AzDownState = 0;  
int ElUpState = 0;
int ElDownState = 0; 
#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(15, 13, 12, 14);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(17);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

void setup() {
  // put your setup code here, to run once:
  pinMode(AzStepperDir, OUTPUT);
  pinMode(AzStepperStep, OUTPUT);
  pinMode(ElStepperDir, OUTPUT);
  pinMode(ElStepperStep, OUTPUT);
  pinMode(AzUp, INPUT);
  pinMode(AzDown, INPUT);
  pinMode(ElUp, INPUT);
  pinMode(ElDown, INPUT);
  Serial.begin(115200); //initializing serial communication
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

}

void loop() {
  AzUpState = digitalRead(AzUp);
  AzDownState = digitalRead(AzDown);
  ElUpState = digitalRead(ElUp);
  ElDownState = digitalRead(ElDown);
  // put your main code here, to run repeatedly:
  if(AzUpState == HIGH) {
    Serial.println("Az Up");
    digitalWrite(AzStepperDir, HIGH);
    digitalWrite(AzStepperStep, HIGH);
    delayMicroseconds(2000);
    digitalWrite(AzStepperStep, LOW);
    delayMicroseconds(2000);
  }
  if (AzDownState == HIGH) {
    Serial.println("Az Down");
    digitalWrite(AzStepperDir, LOW);
    digitalWrite(AzStepperStep, HIGH);
    delayMicroseconds(2000);
    digitalWrite(AzStepperStep, LOW);
    delayMicroseconds(2000);    
  }
  if (ElUpState == HIGH) {
    Serial.println("El Up");
    digitalWrite(ElStepperDir, HIGH);
    digitalWrite(ElStepperStep, HIGH);
    delayMicroseconds(2000);
    digitalWrite(ElStepperStep, LOW);
    delayMicroseconds(2000);   
  }
  if (ElDownState == HIGH) {
    Serial.println("El Down");
    digitalWrite(ElStepperDir, LOW);
    digitalWrite(ElStepperStep, HIGH);
    delayMicroseconds(2000);
    digitalWrite(ElStepperStep, LOW);
    delayMicroseconds(2000);   
  }
  if(ElDownState == LOW && ElUpState == LOW && AzDownState == LOW && AzUpState == LOW) {  
  uint16_t rtd = thermo.readRTD();
  
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

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
  }
  Serial.println();
  delay(500);
  }
}
