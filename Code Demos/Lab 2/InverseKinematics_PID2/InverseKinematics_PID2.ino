// Redbot Code Used
#include <RedBot.h>
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A5, A0);  // initializes encoder on correct pins

// Configuration Space Coordinates
float xPos=0.0; // x position (+forward back-)
float yPos=0.0; // y position (-left right+)
float hPos=0.0; // heading angle (+CW, CCW-)
float lAng=0.0; // left wheel angle in radians
float rAng=0.0; // right wheen anle in radians 
float dHeading=0.0; // change in heading
float dX=0.0; // change in X 
float dY=0.0; // change in Y

// MoIRA Properties and Math
float wheelD = 68.0; // mm 
float wheelC=(wheelD*PI); // wheel circumference, or length driven by one full rotation 
float bodyW=240.0; // Diameter from wheel to wheel 
float turnC=(2*bodyW*PI); // circumference of circle formed by driving a single wheel  
int countsPerRev=192;  // encoder count per wheel revolution, 48:1 gearing, 4 pairs of poles (8 total), 48*8=192
float radsPerCount=(2*PI)/((countsPerRev*turnC)/wheelC); //gives the number of radians in heading change from one of the wheels going forward one count (assuming the other is stationary)

//Encoder count variables 
int lCount=0; //left encoder count
int lPrev=0; //previous encoder reading
int rCount=0; // right encoder count 
int rPrev=0; //previous encoder reading 
int tCount=0; // target encoder count
float mCount=0.0; // mean encoder count
float lDrove=0.0;
float rDrove=0.0;

//Motor properties 
int pwmMax = 200; // max PWM to be allowed for motors 
int pwnNominal=100; // average desired speed
//// PID THINGS ////
float lErr=0;
float rErr=0;
float lErrIntegral;
float rErrIntegral;
float lErrDerivative;
float rErrDerivative;
int velNom=0; // mm/s
int lDrive;
int rDrive;
int driveState=0; // 0 if brake, 1 if driving 
////// PID values
float kP=5;
float kI=0.50;
float kD=0.1;

// Sensors and Buttons
int buttonPin = 12; //On board button
int lBump=11; // left bumper pin
int rBump=3; // right bumper pin

void setup() {
  // put your setup code here, to run once:
  //Configure Inputs
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(lBump, INPUT_PULLUP);
  pinMode(rBump, INPUT_PULLUP);

  // Serial Communication 
 initializeSerial();
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonPressOnBoard(); // wait for button press to begin algorithm 
}

void buttonPressOnBoard(){ 
  if (digitalRead(buttonPin) == LOW)
  {
    report();
    delay(500);
    driveStraight(1000); // drive straight 1000 mm, no PID 
    motors.brake();
    delay(500);
  }
}

void driveStraight(int mmDistance){
  // start driving
  encoder.clearEnc(BOTH);  // Reset the counters.
  tCount=countsPerRev*(mmDistance/wheelC);
  // loop through functions while distance not reached
  while ((lCount <= (tCount)) && (rCount <= (tCount))){
    // non-blocking functions go here 
    encoderCount(); // 
    inverseKinematics();
    pidDrive(pwnNominal, 1, 1);
    report(); //    
  }
}

void pidDrive(int pwnNom, int lDirection, int rDirection){
  //MATH
  lErrDerivative=lErr; //log old value
  rErrDerivative=rErr; //log old value
  mCount=((lCount+rCount)/2);
  lErr=mCount-(lCount);
  rErr=mCount-(rCount);
  // Integral as sum
  lErrIntegral=lErrIntegral+lErr;
  rErrIntegral=rErrIntegral+rErr;
  //Derivative as difference
  lErrDerivative=lErr-lErrDerivative;
  rErrDerivative=rErr-rErrDerivative;

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

  motors.leftDrive(lDirection*lDrive);        // Start driving forward
  motors.rightDrive(rDirection*rDrive);        // Start driving forward
}

void inverseKinematics(){
  dHeading=radsPerCount*((lCount-lPrev)-(rCount-rPrev)); // calculate the change in heading from the relative wheel rotations 
  lDrove=wheelD*PI*float(lCount-lPrev)/float(countsPerRev);
  rDrove=wheelD*PI*float(rCount-rPrev)/float(countsPerRev);
  hPos=hPos+dHeading; // add change in heading to previous heading 
  dX=cos(hPos)*(lDrove+rDrove)/2;
  dY=sin(hPos)*(lDrove+rDrove)/2;
  xPos=xPos+dX;
  yPos=yPos+dY;
}

void encoderCount(){ //2nd 
  // store the encoder counts to a variable.
  lPrev=lCount; //store last encoder reading 
  rPrev=rCount; 
  rCount = encoder.getTicks(LEFT);    // read the left motor encoder NOTE THAT THEY ARE FLIPPED BECAUSE THE BOARD IS BACKWARDS
  lCount = encoder.getTicks(RIGHT);   // read the right motor encoder NOTE THAT THEY ARE FLIPPED BECAUSE THE BOARD IS BACKWARDS
}

void report(){ // 4th
    // print out to Serial Monitor the left and right encoder counts.
  Serial.print("tCount:");
  Serial.print(tCount);
  Serial.print("\t");
  Serial.print("lCount:");
  Serial.print(lCount);
  Serial.print("\t");
  Serial.print("rCount:");
  Serial.print(rCount);
  Serial.print("\t");

  Serial.print("\t");
  Serial.print("lErr:");
  Serial.print(lErr);
  Serial.print("\t");
  Serial.print("lInt:");
  Serial.print(lErrIntegral);
  Serial.print("lDer:");
  Serial.print(lErrDerivative);
  Serial.print("\t");
  Serial.print("rErr:");
  Serial.print(rErr);
  Serial.print("\t");
  Serial.print("rInt:");
  Serial.print(rErrIntegral);
  Serial.print("rDer:");
  Serial.print(rErrDerivative);
  Serial.print("\t");

  Serial.print("\t");
  Serial.print("xPos:");
  Serial.print(xPos);
  Serial.print("\t");
  Serial.print("yPos:");
  Serial.print(yPos);
  Serial.print("\t");
  Serial.print("hPos:");
  Serial.print(hPos);
  Serial.println("\t");
}

void initializeSerial(){
  // Serial Communication 
 Serial.begin(9600);
 Serial.print("INITIALIZE PROGRAM:");
 Serial.print("\t");
 Serial.print("Wheel Distance: ");
 Serial.print(bodyW);
 Serial.print("\t");
 Serial.print("Wheel Diameter:");
 Serial.print(wheelD);
 Serial.print("\t");
 Serial.print("Counts Per Revolution:");
 Serial.print(countsPerRev);
 Serial.print("\t");

 Serial.print("\t");
 Serial.print("kP:");
 Serial.print(kP);
 Serial.print("\t");
 Serial.print("kI:");
 Serial.print(kI);
 Serial.print("\t");
 Serial.print("kD:");
 Serial.print(kD);
 Serial.print("\t");

 Serial.print("\t");
 Serial.println("Press Button to Start");
}