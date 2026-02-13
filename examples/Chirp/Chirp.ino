#include <MoIRA.h>

MoIRA moira(9); // declare 9v battery, 

void setup()
{
  moira.begin();
}

void loop()
{
  moira.chirp(261); // Play C4 note
  delay(250);
  moira.chirp(294); // Play D4 note
  delay(250);
  moira.chirp(330); // Play E4 note
  delay(250);
  moira.chirp(349); // Play F4 note
  delay(250);
  moira.chirp(392); // Play G4 note
  delay(250);
  moira.chirp(440); // Play A4 note
  delay(250);
  moira.chirp(494); // Play B4 note
  delay(250);
  moira.chirp(523); // Play C5 note
  delay(250);
  moira.silence(); // Stop making noise
  delay(1000);
}
