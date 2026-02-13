/*
  MoIRA.h - Library for the Modular Integration Robotics Architecture aka MoIRA.
  Created by Dr. Ada-Rhodes Wish 2/12/2026.
  Released into the public domain.
*/
#ifndef MoIRA_h
#define MoIRA_h

#include <Arduino.h>
#include <RedBot.h> // sparkfun redbot library for board firmware 

class MoIRA
{
  public:
    MoIRA(int batteryV, int configuration);
    void begin();
    void led(int power);
	void fwd(int mmX, int power);
	void rev(int mmX, int power);
	void drive(int powL, int powR);
	void brake();
	int lBump();
	int rBump();
	void chirp(int freq=440);
	void silence();
  private:
    int _ledPin;
	int _buzzPin;
	int _maxMotor;
};

#endif