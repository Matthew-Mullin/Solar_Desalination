//(C) 2011 Hannes Hassler. 
//updated and checked 2015
//Modified by Matthew Mullin 2019

#include "Wire.h"
#include <Helios.h>
#include <DS1107H.h>
#include <TimeLib.h>
#include <Servo.h>

Servo myservoAz;  // create servo object to control a servo
Servo myservoEl;  // create servo object to control a servo
int posOffsetAz=8;
int posOffsetEl = 2;

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 


Helios helios;
DS1107H clock;


double dAzimuth;
double dElevation;
      

void setup()
{
  myservoAz.attach(10);  // attaches the servo on pin 9 to the servo object
  myservoEl.attach(9);  // attaches the servo on pin 10 to the servo object
  Serial.begin(9600);
  while (!Serial) ; // Needed for Leonardo only
  pinMode(13, OUTPUT);
  setSyncProvider( requestSync);  //set function to call when sync required
  Serial.println("Waiting for sync message"); 
  
  
  //uncomment following to set the timer chip
  //Once done, don't forget to comment out again, otherwise
  //you get the same time with each startup ;-)
  /*
  byte second=0;
  byte minute=12;
  byte hour=19;
  byte dayOfWeek=2;
  byte dayOfMonth=1;
  byte month=3;
  byte year=11;
  clock.setDate(second,minute,hour,dayOfWeek,dayOfMonth,month,year);    
  */
  
}



void loop()
{
if (Serial.available()) {
    processSyncMessage();
  }
  if (timeStatus()!= timeNotSet) {
    digitalClockDisplay();  
    if (dAzimuth < 180) {
    myservoAz.write(dAzimuth+posOffsetAz);
    myservoEl.write(dElevation+posOffsetEl);
    } else {
    myservoAz.write(dAzimuth+posOffsetAz-180);
    myservoEl.write((dElevation+posOffsetEl));
    Serial.print(dAzimuth+posOffsetAz-180);
    Serial.print(" dAzimuth ");    
    Serial.print((dElevation+posOffsetEl));
    Serial.print(" dElavation");
    }
  }
  if (timeStatus() == timeSet) {
    digitalWrite(13, HIGH); // LED on if synced
  } else {
    digitalWrite(13, LOW);  // LED off if needs refresh
  }
  delay(1000*5);
}

//16.36667,48.2 is for 48°12'N, 16°22'O (latitude and longitude of Vienna)
/*clock.getDate();  
helios.calcSunPos(clock.year,clock.month,clock.dayOfMonth,
clock.hour, clock.minute,clock.second,16.36667,48.2); 

showTime(clock);
dAzimuth=helios.dAzimuth;show("dAzimuth",dAzimuth,true);
dElevation=helios.dElevation;show("dElevation",dElevation,true);

delay(5000);
  
}*/

void digitalClockDisplay(){
  // digital clock display of the time 
  //16.36667,48.2 is for 48°12'N, 16°22'O (latitude and longitude of Vienna)
  //40.925690 N -73.141129 O
  helios.calcSunPos(year(),month(),day(),
hour(), minute(),second(),-73.141129,40.925690);
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
  dAzimuth=helios.dAzimuth;
  Serial.print("dAzimuth ");
  Serial.print(dAzimuth);
  dElevation=helios.dElevation;
  Serial.print(" dElevation ");
  Serial.print(dElevation);
  Serial.println();
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

/*void show(char nameStr[], double val, boolean newline) {
  Serial.print(nameStr);  
  Serial.print("=");
  if (newline)
       Serial.println(val);
  else Serial.print(val);
}*/

/*void showTime(DS1107H timerChip) {
  Serial.print("UT ");
  Serial.print(timerChip.hour, DEC);
  Serial.print(":");
  Serial.print(timerChip.minute, DEC);
  Serial.print(":");
  Serial.print(timerChip.second, DEC);
  Serial.print("  ");
  Serial.print(timerChip.month, DEC);
  Serial.print("/");
  Serial.print(timerChip.dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(timerChip.year, DEC);
  Serial.print("  Day_of_week:");
  Serial.println(timerChip.dayOfWeek, DEC);  
}*/


