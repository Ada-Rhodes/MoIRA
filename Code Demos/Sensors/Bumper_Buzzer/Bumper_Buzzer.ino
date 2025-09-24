#include <RedBot.h>
RedBotMotors motors;

// Create a couple of constants for our pins.
const int buzzerPin = 11;
const int buttonPin = 12;
const int lBumper = 10;
const int rBumper = 9;

void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP); // configures the button as an INPUT
  pinMode(lBumper, INPUT_PULLUP); // configures the button as an INPUT
  pinMode(rBumper, INPUT_PULLUP); // configures the button as an INPUT
  
  // INPUT_PULLUP defaults it to HIGH.
  pinMode(buzzerPin, OUTPUT);  // configures the buzzerPin as an OUTPUT

  // Serial Communication 
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ( digitalRead(lBumper) == LOW &&  digitalRead(rBumper) == HIGH ){ 
    tone(buzzerPin, 523);   // Play a 1kHz tone on the pin number held in
  }
  else if(digitalRead(lBumper) == HIGH &&  digitalRead(rBumper) == LOW ){
    tone(buzzerPin, 789);
  }
  else if(digitalRead(lBumper) == LOW &&  digitalRead(rBumper) == LOW ){
    tone(buzzerPin, 880);
  }
  else{
    noTone(buzzerPin);       // Stop playing the tone.
  }

  report();
}

void report(){
  //print serial 
  Serial.print("left_Bumper:");
  Serial.print(digitalRead(lBumper));
  Serial.print("\t");
  Serial.print("right_Bumper:");
  Serial.println(digitalRead(rBumper));
}
