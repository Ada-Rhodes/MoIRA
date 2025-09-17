/***********************************************************************
 * Exp7_1_RotaryEncoder -- RedBot Experiment 7_1
 * 
 * Knowing where your robot is can be very important. The RedBot supports
 * the use of an encoder to track the number of revolutions each wheels has
 * made, so you can tell not only how far each wheel has traveled but how
 * fast the wheels are turning.
 * 
 * This sketch was written by SparkFun Electronics, with lots of help from 
 * the Arduino community. This code is completely free for any use.
 * 
 * 8 Oct 2013 M. Hord
 * Revised, 31 Oct 2014 B. Huang 
 ***********************************************************************/
// INCLUDE THE LIBRARY
#include <RedBot.h>
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A0, A5);  // initializes encoder on pins A2 and 10

// DECLARE VARIABLES
int buttonPin = 12; // on board push button so we can tell it to start, NOTE it is an INT because its an integer value
int countsPerRev = 192;   // encoder count per revolution: 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
int lCount; // variables used to store the left encoder counts.
int rCount; // variables used to store the right encoder counts.
float wheelD=(65); // Wheel is 65mm wide, NOTE that it is a float because we want to do decimal math with it
float wheelC=(wheelD*PI); // Calculate wheel circumference 
float bodyW=(240); // center wheel to center wheel width is 240mm 
float turnC=2*bodyW*PI; // circumference of circle formed by driving a single wheel 
//// THIS IS HOW FAR IT WILL GO ////
float driveD; //will drive forward 1 m 
float nRevs;
int nCount=1;
//// PID THINGS ////
float encMean;
float lErr=0;
float rErr=0;
float lErrIntegral;
float rErrIntegral;
float lErrDerivative;
float rErrDerivative;
int pwmNominal=0;
int lDrive;
int rDrive;
////// PID values
float kP=0;
float kI=0;
float kD=0;

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600); //establishes serial communication so we can debug 
  Serial.print("target");
  Serial.print("\t");
  Serial.print("lCount");
  Serial.print("\t");
  Serial.print("rCount");
  Serial.print("\t");
  Serial.print("encMean");
  Serial.print("\t");
  Serial.print("lErr");
  Serial.print("\t");
  Serial.print("rErr");
  Serial.print("\t");
  Serial.print("lErrInt");
  Serial.print("\t");
  Serial.print("rErrInt");
  Serial.print("\t");
  Serial.print("lErrDer");
  Serial.print("\t");
  Serial.print("rErrDer");
  Serial.print("\t");
  Serial.print("lDrive");
  Serial.print("\t");
  Serial.println("rDrive");
}

void loop(void)
{
  // wait for a button press to start driving.
  if (digitalRead(buttonPin) == LOW)
  {
    encoder.clearEnc(BOTH);  // Reset the counters.
    nCount=startPID(1000,100,1,0,0); // function that sets distance in mm and PWM intensity
  }

  // store the encoder counts to a variable.
  lCount = encoder.getTicks(LEFT);    // read the left motor encoder
  rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

  pidForward(pwmNominal);
  
  // print out to Serial Monitor the left and right encoder counts.
  report();

  

  // if either left or right motor are more than 5 revolutions, stop
  if ((lCount >= (nCount)) || (rCount >= (nCount)) )
  {
    motors.brake();
    delay(10000);
    pwmNominal=0;
  }
}

// Follow-Up Questions:
//// What if I want a function that says how far and how high to drive the motors?
int startPID(float distance, int pwmNom, float kProportional, float kIntegral, float kDerivative){
  //update values
  lErrIntegral=0;
  rErrIntegral=0;
  pwmNominal=pwmNom;
  kP=kProportional;
  kI=kIntegral;
  kD=kDerivative;

  //convert to count 
  driveD = distance; //will drive forward 1 m 
  nRevs=(driveD/wheelC); //converts drive distance to # wheel revoluations
  return nCount=int(nRevs*countsPerRev); //convert float revs to int number of clicks 
}

void pidForward(int pwnNom){
  //MATH
  lErrDerivative=lErr; //log old value
  rErrDerivative=rErr; //log old value
  encMean=((lCount+rCount)/2);
  lErr=encMean-(lCount);
  rErr=encMean-(rCount);
  // Integral as sum
  lErrIntegral=lErrIntegral+lErr;
  rErrIntegral=rErrIntegral+rErr;
  //Derivative as difference
  lErrDerivative=lErrDerivative-lErr;
  rErrDerivative=rErrDerivative-rErr;

  lDrive=pwnNom+(kP*lErr)+(kI*lErrIntegral)+(kD*lErrDerivative);
  rDrive=pwnNom+(kP*rErr)+(kI*rErrIntegral)+(kD*rErrDerivative);
  
  // Max and Min Drive
  if(lDrive>=255){
    lDrive=255;
  }
  if(lDrive<=0){
    lDrive=0;
  }
  if(rDrive>=255){
    rDrive=255;
  }
  if(rDrive<=0){
    rDrive=0;
  }

  motors.leftDrive(lDrive);        // Start driving forward
  motors.rightDrive(rDrive);        // Start driving forward
}

void report(){
    // print out to Serial Monitor the left and right encoder counts.
  Serial.print(nCount);
  Serial.print("\t");
  Serial.print(lCount);
  Serial.print("\t");
  Serial.print(rCount);
  Serial.print("\t");
  Serial.print(encMean);
  Serial.print("\t");
  Serial.print(lErr);
  Serial.print("\t");
  Serial.print(rErr);
  Serial.print("\t");
  Serial.print(lErrIntegral);
  Serial.print("\t");
  Serial.print(rErrIntegral);
  Serial.print("\t");
  Serial.print(lErrDerivative);
  Serial.print("\t");
  Serial.print(rErrDerivative);
  Serial.print("\t");
  Serial.print(lDrive);
  Serial.print("\t");
  Serial.print(rDrive);
  Serial.println("\t");
}