#include "MeOrion.h"

#include <Wire.h>
#include <VL53L0X.h>
#include "MeOrion.h"
  
VL53L0X sensor;


//------------------ DC Motor section -------------------
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
uint8_t motorSpeed = 150;

//------------------ DC Motor section -------------------


//------------------ Ultrasonic sensor section -------
MeUltrasonicSensor ultraSensor(PORT_3); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
int wallDistance = 20;

//------------------ Ultrasonic sensor section -------


void setup()
{
  
  Serial.begin(9600);
    
  Wire.begin();
  
  sensor.init();
  sensor.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(100);
  
}

void loop()
{
  
  int headDistance = sensor.readRangeContinuousMillimeters() / 10; //1cm = 10mm

  const int turnDelay = 800; //how long to wait for the rover to do a 90 degree turn
  const int nudgeDelay = 100;  

  
  //hugTheWall(); //always stay close to the right wall
  
  if (turnRightAllowed()) { //can I turn right?
      delay(nudgeDelay); //initial delay to clear the corner
      turnRight(turnDelay); //yes!
      goForward();
      delay(turnDelay); //secondary delay to get into the turn
  } 
  else { //no, cannot turn right
      if (headDistance > wallDistance) { //I can't turn right but can I go straight?
        //yes I can, let's go!
        goForward();
      } 
      else { //no, I cannot go straight OR right, let's go left
              turnLeft(turnDelay);
              goForward();
           }
     }
     //delay(500);
}

void stopEngines() {
    motor1.run(0); 
    motor2.run(0); 
}

void goForward() {
  motor1.run(-motorSpeed); 
  motor2.run(motorSpeed); 
}

boolean turnRightAllowed() {
  int currentDistance = ultraSensor.distanceCm();
  delay(100); //wait for the ping to come back
  if (currentDistance > wallDistance*2) { //can I go right?
    return true;
  } else {
    return false;
  }
}

void turnRight(int delayValue) {
  motor1.run(-motorSpeed);
  motor2.run(-motorSpeed);
  delay(delayValue);
}

void turnLeft(int delayValue) {
  motor1.run(motorSpeed);
  motor2.run(motorSpeed);
  delay(delayValue);
}
