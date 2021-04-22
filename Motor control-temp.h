// Motor A connections -ethl gly.
int enA = ; // !! fix this pin - to pwm input pin
int in1 = 7;
int in2 = 8;
// Motor B connections - seawater.
int enB = ; // !! fix this pin - to pwm input pin
int in3 = 2;
int in4 = 4;


// speed control for Seawater
void speedControl() {
	// Turn on motor B - seawater
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
 
analogWrite(enB, 40); // motor speed to control seawater

}

// speed control for Ethlyene glycol
void loop() {
	// Turn on motor A -ethl gly
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);

  
 // temperature_1 = ethyl gly temperature
 
  if (temperature_1>=160){
  analogWrite(enA, 90);
 
  delay(1000);

}
else if (temperature_1<160){
  analogWrite(enA, 60);
  
  delay(1000);
 
}
 
 }
