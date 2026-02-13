#include <MoIRA.h>

MoIRA moira(9); // declare 9v battery, 

void setup()
{
  moira.begin();
}

void loop()
{
  // Dot - Dot - Dot
  moira.led(HIGH);
  moira.chirp();
  delay(100);
  moira.silence();
  moira.led(LOW);
  delay(100);

  moira.led(HIGH);
  moira.chirp();
  delay(100);
  moira.silence();
  moira.led(LOW);
  delay(100);

  moira.led(HIGH);
  moira.chirp();
  delay(100);
  moira.silence();
  moira.led(LOW);
  delay(250);

  // Dash - Dask - Dash
  moira.led(HIGH);
  moira.chirp();
  delay(500);
  moira.silence();
  moira.led(LOW);
  delay(100);

  moira.led(HIGH);
  moira.chirp();
  delay(500);
  moira.silence();
  moira.led(LOW);
  delay(100);

  moira.led(HIGH);
  moira.chirp();
  delay(500);
  moira.silence();
  moira.led(LOW);
  delay(250);

  // Dot - Dot - Dot
  moira.led(HIGH);
  moira.chirp();
  delay(100);
  moira.silence();
  moira.led(LOW);
  delay(100);

  moira.led(HIGH);
  moira.chirp();
  delay(100);
  moira.silence();
  moira.led(LOW);
  delay(100);

  moira.led(HIGH);
  moira.chirp();
  delay(100);
  moira.silence();
  moira.led(LOW);
  delay(1000);
}