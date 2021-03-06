//
//Hannes Hassler, 1-3-2011
//adapted as library from
//Maurice Ribble
// 4-17-2008
// http://www.glacialwanderer.com/hobbyrobotics
//Hannes Hassler, 22-7-2012
//adaption to Arduino 1.0.1
//as pointed out by Roman Foltyn



//if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif

#include "Wire.h"
#include "DS1107H.h"

#define DS1307_I2C_ADDRESS 0x68

byte nullByte=0;

DS1107H::DS1107H() 
{ 
   Wire.begin();
   
   second;        // 0-59
   minute;        // 0-59
   hour;          // 1-23
   dayOfWeek;     // 1-7
   dayOfMonth;    // 1-28/29/30/31
   month;         // 1-12
   year;
   
 }

// Convert normal decimal numbers to binary coded decimal
byte DS1107H::decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte DS1107H::bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

// Stops the DS1307, but it has the side effect of setting seconds to 0
// Probably only want to use this for testing
/*void stopDs1307()
{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.send(0);
  Wire.send(0x80);
  Wire.endTransmission();
}*/

// 1) Sets the date and time on the ds1307
// 2) Starts the clock
// 3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers
void DS1107H::setDate(
                   byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99
{
   Wire.beginTransmission(DS1307_I2C_ADDRESS);   
   Wire.write(nullByte);
   Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
}

// Gets the date and time from the ds1307
void DS1107H::getDate()

{

  // Reset the register pointer

  Wire.beginTransmission(DS1307_I2C_ADDRESS);

  Wire.write(nullByte);

  Wire.endTransmission();



  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);



  // A few of these need masks because certain bits are control bits

  second     = bcdToDec(Wire.read() & 0x7f);

  minute     = bcdToDec(Wire.read());

  hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm

  dayOfWeek  = bcdToDec(Wire.read());

  dayOfMonth = bcdToDec(Wire.read());

  month      = bcdToDec(Wire.read());

  year       = bcdToDec(Wire.read());

}
