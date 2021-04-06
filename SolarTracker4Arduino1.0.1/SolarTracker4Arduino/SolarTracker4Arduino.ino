//(C) 2011 Hannes Hassler. 
//updated and checked 2015

#include "Wire.h"
#include <Helios.h>
//#include <DS1107H.h>


Helios helios;
//DS1107H clock;


double dAzimuth;
double dElevation;
      

void setup()
{
  Serial.begin(9600); 
  
  
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


//16.36667,48.2 is for 48°12'N, 16°22'O (latitude and longitude of Vienna)
clock.getDate();  
helios.calcSunPos(clock.year,clock.month,clock.dayOfMonth,
clock.hour, clock.minute,clock.second,16.36667,48.2); 

showTime(clock);
dAzimuth=helios.dAzimuth;show("dAzimuth",dAzimuth,true);
dElevation=helios.dElevation;show("dElevation",dElevation,true);

delay(5000);
  
}



void show(char nameStr[], double val, boolean newline) {
  Serial.print(nameStr);  
  Serial.print("=");
  if (newline)
       Serial.println(val);
  else Serial.print(val);
}

void showTime(DS1107H timerChip) {
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
}
