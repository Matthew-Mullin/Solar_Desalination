/*
  Helios.h - Library for calculating the solar
  position.
  Created by Hannes Hassler, February, 2011.  
  modified 22-7-2012,  adaption to Arduino 1.0.1 as pointed out by Roman Foltyn
*/
#ifndef DS1107H_h
#define DS1107H_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


class DS1107H
{
  public:
    DS1107H();   
    byte decToBcd(byte val);
    byte bcdToDec(byte val);
    
    byte second;        // 0-59
    byte minute;       // 0-59
    byte hour;          // 1-23
    byte dayOfWeek;     // 1-7
    byte dayOfMonth;    // 1-28/29/30/31
    byte month;         // 1-12
    byte year;
    
    void setDate(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year);
	
   void getDate();	  
  	
  
   
};




#endif
