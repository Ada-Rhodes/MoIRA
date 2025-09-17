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
float driveD = 1000; //will drive forward 1 m 
float nRevs=(driveD/wheelC);

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600); //establishes serial communication so we can debug 
  Serial.println("target    left    right"); // headings 
  Serial.println("======================="); // line 
}

void loop(void)
{
  // wait for a button press to start driving.
  if (digitalRead(buttonPin) == LOW)
  {
    encoder.clearEnc(BOTH);  // Reset the counters.
    motors.drive(150);        // Start driving forward.
  }

  // store the encoder counts to a variable.
  lCount = encoder.getTicks(LEFT);    // read the left motor encoder
  rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

  // print out to Serial Monitor the left and right encoder counts.
  Serial.print(nRevs*countsPerRev);
  Serial.print("\t");
  Serial.print(lCount);
  Serial.print("\t");
  Serial.println(rCount);

  // if either left or right motor are more than 5 revolutions, stop
  if ((lCount >= (nRevs*countsPerRev)) || (rCount >= (nRevs*countsPerRev)) )
  {
    motors.brake();
  }
}

// Follow-Up Questions:
//// What if I want a function that says how far and how high to drive the motors?
//// The encoders are showing its not driving straight, how can we fix that (using PID)
//// How can we tune it 