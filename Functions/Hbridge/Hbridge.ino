/*  
  L298N Motor Demonstration
  L298N-Motor-Demo.ino
  Demonstrates functions of L298N Motor Controller
  
  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/
  
 
// Motor A
int enA = 34;
int in1 = 32;
int in2 = 33;
const int channelA = 0;
const int frequency = 5000;
const int resolution = 8;
// Motor B
//const int channelB = 1;
//int enB = 25;
//int in3 = 26;
//int in4 = 27;

void setup() 
{
  // Set all the motor control pins to outputs
  //pinMode(enA, OUTPUT);
  ledcSetup(channelA,frequency,resolution);
  ledcAttachPin(enA,channelA);
  //pinMode(enB, OUTPUT);
  //ledcSetup(channelB,frequency,resolution);
  //ledcAttachPin(enB,channelB);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //pinMode(in3, OUTPUT);
  //pinMode(in4, OUTPUT);
}
 
void demoOne()
{
  // This function will run the motors in both directions at a fixed speed
  // Turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set speed to 200 out of possible range 0~255
  //analogWrite(enA, 200);
  ledcWrite(channelA, 255);
  // Turn on motor B
  //digitalWrite(in3, HIGH);
  //digitalWrite(in4, LOW);
  // Set speed to 200 out of possible range 0~255
  //analogWrite(enB, 200);
  //ledcWrite(channelB, 200);
  delay(2000);
  // Now change motor directions
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, HIGH);  
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, HIGH); 
  //delay(2000);
  
  // Now turn off motors
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, LOW);  
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, LOW);
}
 
void demoTwo()
{
  // This function will run the motors across the range of possible speeds
  // Note that maximum speed is determined by the motor itself and the operating voltage
  // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
//  digitalWrite(in3, LOW);
  //digitalWrite(in4, HIGH); 
  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++)
  {
    ledcWrite(channelA, i);
    //ledcWrite(channelB, i);
    delay(20);
  } 
 
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i)
  {
    ledcWrite(channelA, i);
    //ledcWrite(channelB, i);
    delay(20);
  } 
 
  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, LOW);  
}
 
void loop()
{
  demoOne();
  delay(1000);
  //demoTwo();
  //delay(1000);
    // Now turn off motors
  ledcWrite(channelA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW); 
}
