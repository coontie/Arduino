#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <VL53L0X.h>

VL53L0X frontSensor;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rearMotor = AFMS.getMotor(2);
Adafruit_DCMotor *steeringMotor = AFMS.getMotor(3);

//---------- Setup the servo ----------------
Servo radarServo;

int angle = 0;    // variable to store the servo position
int delta = 20;   // new increment for the servo adjustment, move by this value
//-------------------------------------------

const int distanceArrayMaxSize = 180 / 20 + 1;  // 180 degrees divided by delta + 0th position
int dArray[distanceArrayMaxSize];               // array of distances, indexed by angles
unsigned long previousMillis = 0;               // will store last time the servo was moved
const long interval = 50;                       // interval in milliseconds at which to reposition the servo

void setup() {

  radarServo.attach(10);                        //pin 10
  Serial.begin(9600);                           // set up Serial library at 9600 bps

  Wire.begin();


  //--------Laser ToF section-------------
  frontSensor.init();
  frontSensor.setTimeout(500);
  frontSensor.startContinuous();
  //--------Laser ToF section-------------

  AFMS.begin();  // create with the default frequency 1.6KHz

  //init the array to all 0s
  for (int i = 0; i < distanceArrayMaxSize; i++) {
    dArray[i] = 0;
  }

}

void loop() {
  int index;

  int speed;
  int turn = 0;

  int distance = frontSensor.readRangeContinuousMillimeters();

  //tiny bit of optimization, force 180 degrees into {delta} array slots.
  //180 / {delta} is how many samples we take.
  //Because delta can be negative, we need abs() to avoid negative indices
  index = angle / abs(delta);

  dArray[index] = distance;

  int optimalAngle = calcBestAngle();


  //what as the servo's position when this new reading happened? Map it to the steering value.
  //NOTE: the servo is upside down, the values are reversed!
  //IOW, it goes from 180 on the left to 0 on the right.
  if (optimalAngle < 90) { //should we turn right?
    steeringMotor->run(FORWARD); //turn the wheels right. FORWARD=right
    turn = map(optimalAngle, 0, 90, 150, 0);
    steeringMotor->setSpeed(turn);
    //Serial.println("RIGHT TURN");
  } else {
    steeringMotor->run(BACKWARD); //turn the wheels left
    turn = map(optimalAngle, 90, 180, 0, 150);
    steeringMotor->setSpeed(turn);
    //Serial.println("LEFT TURN");
  }


  if (frontSensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }

  //speed = map(distance, 10, 1100, 0, 80);
  speed = 80;
  goForward(speed);

  //backup if too close to an obstacle but only if within the narrow band
  //between 80 and 100 degrees
  while (distance < 100 && angle < 100 && angle > 80) {
    steeringMotor->run(BACKWARD);
    steeringMotor->setSpeed(130);
    Serial.println("Go backwards! ");
    goBackward();
    distance = frontSensor.readRangeContinuousMillimeters();
    Serial.print("angle: ");
    Serial.println(angle);
  }



  // check to see if it's time to move the servo; that is, if the
  // difference between the current time and last time we moved
  // the servo is bigger than the interval at which you want to
  // move the servo.
  unsigned long currentMillis = millis();

  if ((unsigned long)(currentMillis - previousMillis) >= interval) {

    // increment the servo angle by 'delta'
    // if delta is negative, angle will decrease.
    angle += delta;
    previousMillis = currentMillis;       // save the last time we moved
    radarServo.write(angle);              // tell servo to go to position 'angle'

    if ((angle == 0) || (angle == 180)) { //are we at extreme left or right?
      delta = -delta;                     // yes, reverse the delta
      //delay(5000);
    }

  }


}

void goForward(int fwdSpeed) {
  frontMotor->run(FORWARD);
  frontMotor->setSpeed(fwdSpeed);

  rearMotor->run(FORWARD);
  rearMotor->setSpeed(fwdSpeed);
}


void goBackward() {
  int bwdSpeed = 100;
  frontMotor->run(BACKWARD);
  frontMotor->setSpeed(bwdSpeed);

  rearMotor->run(BACKWARD);
  rearMotor->setSpeed(bwdSpeed);

  //  delay(1000);
}

void goStraight() {
  steeringMotor->setSpeed(0);
}


//best angle is defined as the servo position with the largest distance value
int calcBestAngle() {

  int bestAngle = 0;
  int maxDistance = 0;

  //let's go through all the values in the array..
  for (uint8_t i = 0; i < distanceArrayMaxSize; i++) {
    //..to see if the new value is greater than the max seen thus far
    if (dArray[i] > maxDistance) {
      //yes it is!, save bestAngle and reset maxDistance to the new largest value
      bestAngle = i;
      maxDistance = dArray[i];
    }

  }
  //need to multiply by the abs value of delta to restore the angle
  //if we don't, we're returning the array index
  return bestAngle * abs(delta);
}

