For Installation unzip the two libraries
into ~/sketchbook/libraries; each into its respective
directory. 

Open the sketch SolarTracker4Arduino.ino in your Arduino development GUI, 
it should compile with these two libraries.

To run it, you need additionally 
the DS1107 attached to Arduino as described at
http://www.glacialwanderer.com/hobbyrobotics/?p=12
I found with recent (2015) breakout boards you can omit the
pullup resistors, simply connecting VDD, GND, SDA, SCL worked just fine.

Upload the programm to the Arduino and open
the Serial Monitor. You should get messages like

UT 8:40:8  3/4/11  Day_of_week:5
dAzimuth=138.58
dElevation=26.57


For questions or in case you simply find
it useful you can write to
hannes.mybox@gmail.com
