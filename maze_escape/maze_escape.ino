
#include "MeOrion.h"

//------------------ DC Motor section -------------------
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
uint8_t motorSpeed = 120;
//------------------ DC Motor section -------------------


//------------------ Servo section -------------------
MePort port(PORT_6);
Servo servo1; // create servo object to control a servo 
int16_t servo1pin =  port.pin2();//attaches the servo on PORT_6 SLOT2 to the servo object


int angle = 0;
int delta = 5;
int delayInterval = 100;
//------------------ Servo section -------------------

//------------------ Ultrasonic sensor section -------
MeUltrasonicSensor ultraSensor(PORT_3); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */


void setup()
{
  servo1.attach(servo1pin); // attaches the servo on servopin2
  Serial.begin(9600);
  servo1.write(0);  //position to the beginning
}

void loop()
{
  int bestAngle = 0; // preferred direction of travel

  motor1.run(-motorSpeed); /* value: between -255 and 255. */
  motor2.run(motorSpeed); /* value: between -255 and 255. */
  
  
  bestAngle = sweep(); // get the best direction of travel
  
  Serial.print("Distance : ");
  Serial.println(ultraSensor.distanceCm() );
  
  Serial.print(bestAngle );
  Serial.println(" cm");
  //delay(100); /* the minimal measure interval is 100 milliseconds */

  if (bestAngle >= 90) { //pivot left a bit
    int adjustedSpeed = motorSpeed - bestAngle + 90;
    motor1.run(-adjustedSpeed);
    delay(500);
    motor1.run(-motorSpeed);
  } //end of if

 if (bestAngle < 90) { //pivot right a bit
    int adjustedSpeed = motorSpeed - bestAngle - 90;
    motor2.run(adjustedSpeed);
    delay(500);
    motor2.run(motorSpeed);
  } //end of if


}

int sweep()
{
  int maxDistance = 0;
  int currentDistance = 0;
  int returnAngle;
  
  while ((angle >= 0) && (angle <= 180)) {
    currentDistance = ultraSensor.distanceCm();
 
    if (currentDistance > maxDistance) {
      maxDistance = currentDistance;
      returnAngle = angle;
    }
  
    angle += delta;
  
    servo1.write(angle);                  // sets the servo position according to the scaled value 
  
    delay(delayInterval);
    //Serial.print("angle is");
    //Serial.println(angle);
  } //end of while
  delta = -delta; //flip the direction of servo movement
  angle += delta; //move the servo to get into the while loop on the next run
  return returnAngle;
 
}

