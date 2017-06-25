
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

int angle = 180; //servo angle position
int delta = 30; //step function to move the servo angle by
int currentServoPosition = -1;
//uint8_t delayInterval = 100; //how long to wait for the ping to come back
  
const int dArrayMaxSize = 6; //180 degrees divided by 30 delta = 6 samples per sweep
//const int adjustmentSpeed = 100;
//const int pivotDelay = 600; //how long to wait for the robot to turn
int dArray[dArrayMaxSize]; //array of distances, indexed by angles

int newAngle = 90; // preferred direction of travel

//------------------ Servo section -------------------

//------------------ Ultrasonic sensor section -------
MeUltrasonicSensor ultraSensor(PORT_3); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */


void setup()
{
  servo1.attach(servo1pin); // attaches the servo on servopin2
  Serial.begin(9600);
  servo1.write(angle);  //position to the beginning
//  for (int i=0; i<=dArrayMaxSize; i = i++) {
//    dArray[i] = 0;
//  }
}

void loop()
{

  int adjustedSpeed = 150;
  
  //start the engines!
  motor1.run(-motorSpeed); 
  motor2.run(motorSpeed); 

  if (dArray[currentServoPosition] < 10) { //it's a trap!
    motor1.run(motorSpeed); 
    motor2.run(motorSpeed); 
    delay(300);
  }
  
  currentServoPosition++;                     //increment the array index by 1
  
  //check to see if we are about to go outside the dArrayMaxSize. If yes, then we reset to 0.
  if (currentServoPosition >= dArrayMaxSize) {
    currentServoPosition = 0;
  }
  
  //turn the servo around if at the edge boundary
  if ((angle > 178) || (angle <= 0)){
     delta = -delta;
  }

  
  servo1.write(angle);

  angle += delta; //always add the delta to the angle. Negative delta decreases the angle.

  dArray[currentServoPosition] = ultraSensor.distanceCm(); //get the sensor reading and add it to the array
  //dArray[currentServoPosition] = rand() % 400;
  //Serial.print("current distance reading: ");
  //Serial.println(dArray[currentServoPosition]);
  if (largestDistance(dArray[currentServoPosition])) {
     
      //Serial.print("We have a new angle: ");
      //newAngle = angle;
      //Serial.print(angle);
      //Serial.print("Distance: ");
      //Serial.println(dArray[currentServoPosition]);
      
      if (angle > 90) { //pivot left a bit. NOTE: this is reversed: 180 - 0 degrees!
        //Serial.println("Turning left!");
        //int adjustedSpeed = motorSpeed + angle + adjustmentSpeed;
        //int adjustedSpeed = -50;
        //Serial.print("adjusted speed: ");
        //Serial.println(adjustedSpeed);
        
        motor1.run(adjustedSpeed);
        motor2.run(adjustedSpeed);
        //delay(pivotDelay);
        //motor1.run(-motorSpeed);
        
      } //end of if

      if (angle < 90) { //pivot right a bit
        //Serial.println("Turning right!");
        //int adjustedSpeed = motorSpeed - angle - adjustmentSpeed;
        
        //Serial.print("adjusted speed: ");
        //Serial.println(adjustedSpeed);
        
        motor1.run(-adjustedSpeed);
        motor2.run(-adjustedSpeed);
        //delay(pivotDelay);
        //motor2.run(motorSpeed);
        
      } //end of if
  } //end of if (largestDistance(dArray[currentServoPosition])) {
  
/*
  Serial.print(" angle: ");
  Serial.print(angle);
  Serial.print(" servo position: ");
  Serial.print(currentServoPosition);
  Serial.print(" new angle: ");
  Serial.print(newAngle);
  Serial.print(" distance: ");
  Serial.println(dArray[currentServoPosition]);
*/
  delay(140);
}

boolean largestDistance(int distanceGiven)
{
  int maxDistance = 0;  
  boolean changeCourse = false;
//  Serial.print("getting the max value from the array:");
  for (int i=0; i<dArrayMaxSize; i++) {
    maxDistance = max(maxDistance,dArray[i]);
  }
  /*
  Serial.println();
  Serial.print(" distance given: ");
  Serial.print(distanceGiven);
  Serial.print(" max distance: ");
  Serial.print(maxDistance);
*/
  //if these are equal, that means the newly given value is the current max, hence we are changing course to reflect the new direction
  if (distanceGiven == maxDistance) {
    changeCourse = !changeCourse; //flip this bool from false to true
  }

  return changeCourse;
}


/*
int getMaxValue(int arrayOfDistances[], int arraySize)
{
  int maxDistance = 0;
  int returnAngle = 0;
  for (int i=0; i<=arraySize; i = i++) {
    if (maxDistance > arrayOfDistances[i]) {
      maxDistance = arrayOfDistances[i];
      returnAngle = i;
    }
  }

  return returnAngle;
}

*/
