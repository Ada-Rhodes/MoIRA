#include <MoIRA.h>

MoIRA moira(9,1); // declare 9v battery, standard configuration=1

void setup()
{
  moira.begin();
}

void loop()
{
  moira.drive(0,0);
  delay(500);
  moira.drive(100,0);
  delay(500);
  moira.drive(0,100);
  delay(500);
  moira.drive(100,100);
  delay(500);
  moira.brake();
  moira.drive(-100,-100);
  delay(500);
  moira.drive(0,-100);
  delay(500);
  moira.drive(-100,0);
  delay(500);
  moira.brake();
  delay(500);
}