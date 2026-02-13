/*
  MoIRA.cpp - Library for the Modular Integration Robotics Architecture aka MoIRA.
  Created by Dr. Ada-Rhodes Wish 2/12/2026.
  Released into the public domain.
*/

#include "Arduino.h"
#include "MoIRA.h"
#include <RedBot.h> // sparkfun redbot library for board commands 

RedBotMotors motors; // Instantiate the motor control object in theory I only need to do this once
RedBotBumper lBumper = RedBotBumper(A6); //initialzes bumper object on pin A6
RedBotBumper rBumper = RedBotBumper(A3); // initialzes bumper object on pin A3


MoIRA::MoIRA(int batteryV, int configuration)
{
  _maxMotor=255*5/batteryV;
  if (configuration==1){	  
	  _ledPin = 13; // onboard D13 LED pin 
	  _buzzPin=9; // buzzer installed on pin 9
  } else {
	  _ledPin = 13; // onboard D13 LED pin 
	  _buzzPin=9; // buzzer installed on pin 9
  }
}

void MoIRA::begin()
{
  // this code is called during setup 
  pinMode(_ledPin, OUTPUT); //sets the onboard pin as an output 
  pinMode(_buzzPin, OUTPUT);
}

void MoIRA::led(int power)
{
  digitalWrite(_ledPin, power); //used to turn LED on and off
}

void MoIRA::fwd(int mmX, int power)
{
	int drivePower=power*_maxMotor/100;
	motors.drive(drivePower);
	// I need to add an entire ass control system here 
}

void MoIRA::rev(int mmX, int power)
{
	int drivePower=power*_maxMotor/100;
	motors.drive(-drivePower);
	// I need to add an entire ass control system here 
}

void MoIRA::drive(int powL, int powR)
{
	int drivePowerL=powL*_maxMotor/100;
	int drivePowerR=powR*_maxMotor/100;
	motors.leftMotor(-drivePowerL);
	motors.rightMotor(drivePowerR);
}

void MoIRA::brake()
{
	motors.brake();
}

int MoIRA::lBump()
{
	return lBumper.read();
}

int MoIRA::rBump()
{
	return rBumper.read();
}

void MoIRA::chirp(int freq=440)
{
	tone(_buzzPin, freq); // produce beep of desired hz
}

void MoIRA::silence()
{
	noTone(_buzzPin); // produce beep of desired hz
}