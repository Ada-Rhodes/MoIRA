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
int pwmNominal=100;
int velNom=0; // mm/s
int pwmMax=200;
int lDrive;
int rDrive;
int driveState=0; // 0 if brake, 1 if driving 
////// PID values
float kP=0;
float kI=0;
float kD=0;

void setup() {
  // put your setup code here, to run once:
  // Set button as input with digital pullup 
  pinMode(buttonPin, INPUT_PULLUP);

  // estblish serial communication for debug 
  Serial.begin(9600); //establishes serial communication so we can debug 

  startSerialComm(); // start serial comm
}

void loop() {
  // put your main code here, to run repeatedly:
  // Loop through non-blocking functions
  buttonPressOnBoard(); // 1st check for button press
  encoderCount(); // 2nd update encoder count
  pidForward(driveState, pwmNominal); // 3rd drive motors 
  report(); // 4th print out values for debug 
  checkGoal(); // 5th determine if goal position reached 
}

void buttonPressOnBoard(){ // 1st 
  if (digitalRead(buttonPin) == LOW)
  {
    encoder.clearEnc(BOTH);  // Reset the counters.
    nCount=startPID(10000,velNom,1,0,0); // function that sets distance in mm and PWM intensity
  }
}

void encoderCount(){ //2nd 
  // store the encoder counts to a variable.
  lCount = encoder.getTicks(LEFT);    // read the left motor encoder
  rCount = encoder.getTicks(RIGHT);   // read the right motor encoder
}

void pidForward(int driveSt, int pwnNom){ // 3rd
  if (driveSt==0){
    return;
  } else{
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
    if(lDrive>=pwmMax){
      lDrive=pwmMax;
    }
    if(lDrive<=0){
      lDrive=0;
    }
    if(rDrive>=pwmMax){
      rDrive=pwmMax;
    }
    if(rDrive<=0){
      rDrive=0;
    }

    motors.leftDrive(lDrive);        // Start driving forward
    motors.rightDrive(rDrive);        // Start driving forward
  }
}

void report(){ // 4th
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

void checkGoal(){ //5th 
    // if either left or right motor are more than 5 revolutions, stop
  if ((lCount >= (nCount)) || (rCount >= (nCount)) )
  {
    motors.brake();
    pwmNominal=0;
    velNom=0; 
    driveState=0;
  }
}

// Things below this are sub functions 
int startPID(float distance, int velNom, float kProportional, float kIntegral, float kDerivative){
  //update values
  lErrIntegral=0;
  rErrIntegral=0;
  velNom=500; //mm/s
  pwmNominal=velNom*0.2; // convert nominal velocity to pwm
  kP=kProportional;
  kI=kIntegral;
  kD=kDerivative;
  driveState=1;

  //convert to count 
  driveD = distance; //will drive forward 1 m 
  nRevs=(driveD/wheelC); //converts drive distance to # wheel revoluations
  return nCount=int(nRevs*countsPerRev); //convert float revs to int number of clicks 
}

void startSerialComm(){
    // estblish serial communication for debug 
  Serial.begin(9600); //establishes serial communication so we can debug 

  // Print heading labels for serial communication 
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