#include <MoIRA.h>

MoIRA moira(9); // declare 9v battery, 

void setup()
{
  moira.begin();
}

void loop()
{
  moira.led(HIGH);
  delay(500);
  moira.led(LOW);
  delay(1000);
}