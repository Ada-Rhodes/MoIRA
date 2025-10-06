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
//Kinematic math variables 
float dHeading=0.0; // change in heading
float arcRad=0.0; // radius of arc segment
float cOut=0.0;
float cIn=0.0;

// MoIRA Properties 
float wheelD = 65.0; // mm 
float wheelC=(wheelD*PI); // wheel circumference, or length driven by one full rotation 
float bodyW=240.0; // Diameter from wheel to wheel 
float turnC=(2*bodyW*PI); // circumference of circle formed by driving a single wheel  
int countsPerRev=192;  // encoder count per wheel revolution, 48:1 gearing, 4 pairs of poles (8 total), 48*8=192
int lCount=0; //left encoder count
int lPrev=0; //previous encoder reading
int rCount=0; // right encoder count 
int rPrev=0; //previous encoder reading 
int tCount=0; // target encoder count
float mCount=0; // mean encoder count
float lRevs = 0; // number of left wheel revolutions 
float rRevs = 0; // number of right wheel revolutions 
float mRevs = 0; // mean revolutions between wheels 
int pwmMax = 200; // max PWM to be allowed for motors 
int pwmNominal=75; // average desired speed


// Sensors and Buttons
int buttonPin = 12; //On board button
int lBump=11; // left bumper pin
int rBump=3; // right bumper pin

////// PID values
float kP=1.0;
float kI=0.0;
float kD=0.0;
float lErr=0;
float rErr=0;
float lErrIntegral;
float rErrIntegral;
float lErrDerivative;
float rErrDerivative;


void setup() {
  // put your setup code here, to run once:
  //Configure Inputs
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(lBump, INPUT_PULLUP);
  pinMode(rBump, INPUT_PULLUP);

  // Serial Communication 
 Serial.begin(9600);
 Serial.print("INITIALIZE PROGRAM: Press Button to Start");
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonPressOnBoard(); // wait for button press to begin algorithm 
}

void buttonPressOnBoard(){ // 1st 
  if (digitalRead(buttonPin) == LOW)
  {
    delay(500);
    driveStraight(1000,1,0,0); // function that sets distance in mm and PWM intensity
    delay(500);
  }
}


void driveStraight(int mmDistance, float kProp, float kInt, float kDer){
  // start driving
  encoder.clearEnc(BOTH);  // Reset the counters.
  tCount=countsPerRev*(mmDistance/wheelC);
  // loop through functions while distance not reached
  while ((lCount <= (tCount)) && (rCount <= (tCount))){
    // non-blocking functions go here 
    encoderCount(); // 
    inverseKinematics();
    motors.leftDrive(pwmNominal);
    motors.rightDrive(pwmNominal);
    report(); //  
    checkGoal(); //  
  }
}

void checkGoal(){ //5th 
    // if either left or right motor are more than 5 revolutions, stop
  if ((lCount >= (tCount)) || (rCount >= (tCount)))
  {
    motors.brake();
  }
}

void encoderCount(){ //2nd 
  // store the encoder counts to a variable.
  lPrev=lCount; //store last encoder reading 
  rPrev=rCount; 
  lCount = encoder.getTicks(LEFT);    // read the left motor encoder
  rCount = encoder.getTicks(RIGHT);   // read the right motor encoder
}

void inverseKinematics(){
  lRevs=float(wheelC*(lCount-lPrev)/countsPerRev);
  rRevs=float(wheelC*(rCount-rPrev)/countsPerRev);
  mRevs=(lRevs+rRevs)/2; 
  if(lRevs>rRevs){
    cOut=lRevs;
    cIn=rRevs;
    if (cIn==0.0){
      cIn=0.000000000000000000000000000000001;
    }

    arcRad=(bodyW)/((cOut/cIn)-1); // calculate instantaneous arc radius 
    dHeading=cIn/(2*arcRad);
    
    //Update configuration
    hPos=hPos+dHeading;
    xPos=xPos+cos(hPos)*(mRevs);
    yPos=yPos+sin(hPos)*(mRevs);

  } else if(rRevs>lRevs){
    cOut=rRevs;
    cIn=lRevs;
    if (cIn==0.0){
      cIn=0.000000000000000000000000000000001;
    }

    arcRad=(bodyW)/((cOut/cIn)-1); // calculate instantaneous arc radius 
    dHeading=-1*cIn/(2*arcRad);

    //Update configuration
    hPos=hPos+dHeading;
    xPos=xPos+cos(hPos)*(mRevs);
    yPos=yPos+sin(hPos)*(mRevs);
  } else{
    dHeading=0.0;
    //Update configuration
    hPos=hPos+dHeading;
    xPos=xPos+cos(lRevs)*(mRevs);
    yPos=yPos+sin(lRevs)*(mRevs);
  }
}

void report(){ // 4th
    // print out to Serial Monitor the left and right encoder counts.
  Serial.print(tCount);
  Serial.print("\t");
  Serial.print(lCount);
  Serial.print("\t");
  Serial.print(rCount);
  Serial.print("\t");
  
  Serial.print(xPos);
  Serial.print("\t");
  Serial.print(yPos);
  Serial.print("\t");
  Serial.print(hPos);
  Serial.println("\t");
}