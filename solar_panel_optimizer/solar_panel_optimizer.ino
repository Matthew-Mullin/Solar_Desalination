//URL :: https://devpost.com/software/solar-panel-optimizer
//code includes function from John Leeman, formulas from David Maslanka Ph.D, and code built with J. Colby Cox
//Arduino UNO
#include <math.h> //do math

#include <Time.h> // calculate time
#include <TimeLib.h>

const int weatherinp = 7; //weather input from mkr 1000
int val = 0; //value of input voltage

#include <Servo.h> // include servo library
int servoPinAz = 6; //servo pin for azimuth //bottom
int servoPinEl = 9; //servo pin for elevation //top
Servo servoAz; //initialize servos
Servo servoEl;

const float pi = 3.14;
const float lat = 39.9526; //lattitude
const float lon = -75.1652; //longitude
const float dr = pi / 180; //degrees to radians
const float rd = 190 / pi; //radians to degrees

//time variables
time_t yr; //year
time_t mo; //month
time_t d; //day
time_t h; //hour
time_t m; //minute
int day_of_year; //day of the year (1-365(366))


//solar calculations
double az; //azimuth
double el; // elevation, 90 - Solar Zenith Angle 
double declin; //solar declination
double sha; //solar hour angle
//old versions of angles
double oldAz = 0;
double oldEl = 0;
int azAng = 360; //angle of azimuth
int elAng = 360; //angle of elevation

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600); // begin serial read
setTime(16,30,0,12,12,2016); // set time REMEMBER TO DO THIS
servoAz.attach(servoPinAz); //attach servos
servoEl.attach(servoPinEl);
servoAz.write(0); //initial conditions
servoEl.write(0);

pinMode(weatherinp, INPUT); //set pin 7 to receive input from MKR 1000

}

void loop() {
  val = digitalRead(weatherinp);
  if(val == HIGH){ //only run if the sun could be out (when input voltage is high)
  //find date
  yr = year();
  m = month();
  d = day();
  h = hour();
  m = minute();
  //find day of year
  day_of_year = calculateDayOfYear(d, mo, yr);
//find declination
declin = asin(0.39795 * cos(dr *( 0.98563 * ( day_of_year - 173 ))));
//find solar hour angle
  sha = dr*(15 * (h + (m / 60) -12));
//find solar elevation angle (elevation)
  el = asin(sin(declin) * sin(dr * lat) + cos(declin) * cos(sha) * cos(dr * lat));
//find azimuth
  az = acos((sin(declin) * cos(dr * lat) - cos(declin) * cos(sha) * sin(dr * lat)) / cos(el));
  //modify az and el
  el = floor(rd * el);
  if(sha <= 0){
    az = floor(rd * az);
  }else{
    az = floor(360 - rd * az);
  }

  //see if we need to do anything
  if (el > 0 ) { //sun is up
    if ((az > oldAz + 5) && ((oldEl > oldEl + 5) || (oldEl < oldEl - 5))){ //see if it is worth changing
      azAng = int(az);
      elAng = int(el);
      servoAz.write(azAng); //pivot to azimuth
      delay(100); //delay
      servoEl.write(elAng); //pivot to el
      oldAz = az; //make az old;
      oldEl = el; //make el old;
    }
    
  }else{ //sun is down
    if(azAng != 0){ //skip if we already have done this
      azAng = 0;
      elAng = 0;
      servoAz.write(azAng);
      servoEl.write(elAng);
    }
  }
  

  }
}
int calculateDayOfYear(int day, int month, int year) { //function created by John Leeman (from github) 
  // Given a day, month, and year (4 digit), returns 
  // the day of year. Errors return 999.
  int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  // Verify we got a 4-digit year
  if (year < 1000) {
    return 999;
  }
  // Check if it is a leap year
  if (year%4  == 0) {
    if (year%100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year%400 == 0) {
        daysInMonth[1] = 29;
      }
    }
   }
  // Make sure we are on a valid day of the month
  if (day < 1) 
  {
    return 999;
  } else if (day > daysInMonth[month-1]) {
    return 999;
  }
  int doy = 0;
  for (int i = 0; i < month - 1; i++) {
    doy += daysInMonth[i];
  }
  doy += day;
  return doy;
}
