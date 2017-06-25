
//  Adafruit Arduino - Lesson 13. DC Motor



int motorPin = 3;

void setup()
{
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  while (! Serial);
  //  Serial.println("Speed 0 to 255");
}


void loop()
{
  //    int speed = -255;
  //   analogWrite(motorPin, speed);
  digitalWrite(motorPin, HIGH);

}

/*


#include "MeOrion.h"

//------------------ DC Motor section -------------------
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
uint8_t motorSpeed = 120;
//------------------ DC Motor section -------------------


void setup()
{
  Serial.begin(9600);
}

void loop () {
    //start the engines!
  motor1.run(-motorSpeed); 
  motor2.run(motorSpeed); 

}


*/
