// include the servo library
#include <Servo.h>

Servo myServo;  // create a servo object

int const potPin = A0; // analog pin used to connect the potentiometer
int potVal = 0;  // variable to read the value from the analog pin
int angle = 0;   // variable to hold the angle for the servo motor
int delta = 5
;
int delayInterval = 10;

void setup() {
  myServo.attach(9); // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); // open a serial connection to your computer
}

void loop() {
  //potVal = analogRead(potPin); // read the value of the potentiometer
  // print out the value to the serial monitor
  //Serial.print("potVal: ");
  //Serial.print(potVal);

  // scale the numbers from the pot - 179 is too much??
 // angle = map(potVal, 0, 1023, 0, 150);

  
  if ((angle > 178) || (angle < 0)){
     delta = -delta;
  }

  angle += delta;
    
  // print out the angle for the servo motor
  //Serial.print(" angle: ");
  //Serial.println(angle);

  // set the servo position
  myServo.write(angle);

  // wait for the servo to get there
  delay(delayInterval);
}


