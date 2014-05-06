/**Arduino car that follows a moving target
  Adaptation of wall-avoiding car
  Copyright 2014 Max Bright. All Rights Reserved.
  
  Requires Adafruit Motorshield and NewPing libraries
  
  All distance calculations use meters as units
  */
  
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include "utility/Adafruit_PWMServoDriver.h"

const int sensorPinR = 8;
const int sensorPinL = 7;
const int debugLEDY = 10;
const int debugLEDR = 13;


char carState = 's';

void threadedDelay(long, long);

void startDriving(int);
void stopDriving();
void startTurn(int, char);

//Equations for lines that follow the middle of each sensor's visible cone
float sensorLSlope = -1.76;
float sensorLIntercept = 0.86;
float sensorRSlope = 1.76;
float sensorRIntercept = -0.86;

bool targetFoundHistory[20];

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motor1;
Adafruit_DCMotor *motor2;
Adafruit_DCMotor *motor3;
Adafruit_DCMotor *motor4;

NewPing sensorR = NewPing(sensorPinR, sensorPinR, 300);
NewPing sensorL = NewPing(sensorPinL, sensorPinL, 300);

void setup()
{                
  pinMode(debugLEDY, OUTPUT);
  pinMode(debugLEDR, OUTPUT);
  
  Serial.begin(9600);
  
  
  motor1 = AFMS.getMotor(1);
  motor2 = AFMS.getMotor(2);
  motor3 = AFMS.getMotor(3);
  motor4 = AFMS.getMotor(4);
  AFMS.begin();
  
  for (int i = i; i < ( sizeof(targetFoundHistory)/sizeof(bool) ); i++)
  {
    targetFoundHistory[i] = false;
  }
  
}

/*This ensures program operates on an exact clock, in case certain
  parts of the code take a long time to run while a function should
  maintain the same total duration*/
void threadedDelay(long delayTime, long initialTime = -1)
{
  long initialDelay;
  long loopStartTime;
  long elapsedTime;
  
  
  if (initialTime >= 0)
  {
    initialDelay = millis() - initialTime;
    
    delayTime = delayTime - initialDelay;
  }
  
  loopStartTime = millis();
  elapsedTime = 0;
  delay(delayTime - (millis() - loopStartTime) );
}

//Gets distance of target from sensor
float checkDistance(char dir)
{
  bool result;
  
  long duration;//, cm;
  float cm;
  
  if (dir == 'L')
    cm = sensorL.ping_cm();
  else if (dir == 'R')
    cm = sensorR.ping_cm();
  
  return cm;
}

void startDriving(int motorSpeed = 255)
{
  motor1->setSpeed(motorSpeed);
  motor1->run(FORWARD);
  motor2->setSpeed(motorSpeed);
  motor2->run(FORWARD);
  motor3->setSpeed(motorSpeed);
  motor3->run(FORWARD);
  motor4->setSpeed(motorSpeed);
  motor4->run(FORWARD);
  carState = 'f';
}

void startReversing(int motorSpeed = 255)
{
  motor1->setSpeed(motorSpeed);
  motor1->run(BACKWARD);
  motor2->setSpeed(motorSpeed);
  motor2->run(BACKWARD);
  motor3->setSpeed(motorSpeed);
  motor3->run(BACKWARD);
  motor4->setSpeed(motorSpeed);
  motor4->run(BACKWARD);
  carState = 'r';
}

void stopDriving()
{
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
  carState = 's';
}

//Turns while still moving forward
void startFastTurn(int motorSpeed = 127, float factor = 0.5, char dir = 'r')
{
  float motorSpeedProduct = motorSpeed*factor;
  
  if (motorSpeedProduct > 255)
  {
    Serial.println("Warning: called startFastTurn with a factor too large");
    motorSpeedProduct = 255;
  }
  else if (motorSpeed * factor < 0)
  {
    Serial.println("Warning: called StartFastTurn with a factor too small");
    motorSpeedProduct = 0;
  }
  
  if (dir == 'r')
  {
    motor1->setSpeed(motorSpeed);
    motor2->setSpeed(motorSpeed*factor);
    motor3->setSpeed(motorSpeed*factor);
    motor4->setSpeed(motorSpeed);
  }
  else
  {
    motor1->setSpeed(motorSpeed*factor);
    motor2->setSpeed(motorSpeed);
    motor3->setSpeed(motorSpeed);
    motor4->setSpeed(motorSpeed*factor);
  }
  
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor3->run(FORWARD);
  motor4->run(FORWARD);
  
  carState = 'm';
}

//Turns by pivoting
void startTurn(int motorSpeed = 127, char dir = 'r')
{
  motor1->setSpeed(motorSpeed);
  motor2->setSpeed(motorSpeed);
  motor3->setSpeed(motorSpeed);
  motor4->setSpeed(motorSpeed);
  
  if (dir == 'r')
  {
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);
    motor4->run(FORWARD);
  }
  else
  {
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(BACKWARD);
  }
  
  
  carState = 't';
}

//Find X intersection between two lines
float findIntersect(float slope1, float intercept1, float slope2, float intercept2)
{
        float result;
 
        intercept1 -= intercept2;
        slope2 -= slope1;
        if (slope2 != 0)
                result = intercept1 / slope2;
        else
        {
                Serial.print("Tried findIntercept on parallel lines\n");
                result = 0;
        }
       
        return result;
}

//Get coordinates of target relative to the car based on sensor data
void getCoords(float* x, float* y, float lDist, float rDist)
{
  float aSlope = .568; //Perpendicular to sensorL function
  float bSlope = -.568; //Perpendicular to selsorR function
  
  float aIntercept = (.06-(lDist*.174))-(lDist*.56);
  float bIntercept = (-.06+(rDist*.174))+(rDist*.56);
  
  *x = findIntersect(aSlope, aIntercept, bSlope, bIntercept);
  *y = aIntercept + (aSlope * *x);
}

void loop()                     
{
  bool badSensorData = false;
  
  unsigned long initialTime;
  
  initialTime = millis();

  float distanceL = checkDistance('L')/100;
  delay(50);
  float distanceR = checkDistance('R')/100;
  
  if (distanceR < 2)
  {
     digitalWrite(debugLEDR, HIGH); 
  }
  else
  {
     digitalWrite(debugLEDR, LOW); 
  }
  
  if (distanceL < 2)
  {
     digitalWrite(debugLEDY, HIGH); 
  }
  else
  {
     digitalWrite(debugLEDY, LOW); 
  }
  
  float x, y;
  getCoords(&x, &y, distanceL, distanceR);

  badSensorData = (!(abs(distanceL - distanceR) < 0.15));
  
  /*Update log of last 20 checks of whether the target was found
  and check for how long no target was found*/
  int targetLostCount = 0;
  bool targetFoundHistoryOld[20];
  memcpy(targetFoundHistoryOld, targetFoundHistory, sizeof(targetFoundHistoryOld));
  bool reachedTargetFound = false;
  targetFoundHistory[0] = (!badSensorData && (x > 0 && x < 1) );
  for (int i = 0; i < ( sizeof(targetFoundHistory)/sizeof(bool) ); i++)
  {
    if (i > 0)
      targetFoundHistory[i] = targetFoundHistoryOld[i-1];
    if (!targetFoundHistory[i] && !reachedTargetFound)
      targetLostCount++;
    else
      reachedTargetFound = true;
  }
  
  
  if (!targetFoundHistory[0])
  {
    /*If lost target for 5 or less loop increments, 
	keep following previous path*/
    if (carState != 's' && targetLostCount > 5)
    {
	  Serial.print("Lost target; targetLostCount = ");
	  Serial.print(targetLostCount);
	  Serial.print(" and targetFoundHistory[1] = ");
	  Serial.print(targetFoundHistory[1]);
	  Serial.print("\n");
	  stopDriving();
    }
  }

  else if (0.2 <= x && x < 1)
  {
    if (y < 0)
    {
	  startFastTurn(255, 0.5, 'r');
    }
    else if (y > 0)
    {
	  startFastTurn(255, 0.5, 'l');
    }
    else
    {
	  startDriving(255);
    }
  }
  else if (x < 0.2)
  {
    if (x > 0.1 && y != 0)
    {
	  if (y < 0)
	    startTurn(127, 'r');
	  else if (y > 0)
	    startTurn(127, 'l');
    }
    else
    {
	  startReversing(127);
    }
  }
  
  threadedDelay(200, initialTime);
}




