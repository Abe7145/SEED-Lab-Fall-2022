/*
 * The Seedsters
 * Created by Scott Reeder
 * 
 * This code implements a closed loop step response for forward motion of the robot. It then prints out time
 * and velocity values to the serial monitor.
 */
 
#include <DualMC33926MotorShield.h>


//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//\\\\\\\ MOTOR 1 == RIGHT MOTOR   \\\\\\\\\\  MOTOR 2 == LEFT MOTOR \\\\\\\\\\\\\\\\\
/////////////////////////////////////////////////////////////////////////////////////
DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
#define MOTORVOLTAGE  400
const int LEFT_PIN_A = 3; 
const int LEFT_PIN_B = 11;
const int RIGHT_PIN_A = 2;
const int RIGHT_PIN_B = 5;

// This program uses a state machine to track the encoder’s position
// These variables store the state
bool A_LAST_STATE = 0;
bool A_CURRENT_STATE_LEFT = 0;
bool B_CURRENT_STATE_LEFT = 0;
bool A_CURRENT_STATE_RIGHT = 0;
bool B_CURRENT_STATE_RIGHT = 0;

int index = 0;

int countLeft; // measures motor encoder counts -- unitless
int countRight;
int intError = 0; //integral error for I in PI controller -- tenths of a degree second       
int prevMyTime = 0; //previous loop time marker -- milliseconds 
int currMyTime = 0; //current time marker -- milliseconds 
//int prev_posLeft = 0; // stores the position of the motor in the previous loop -- position in degrees
//int prev_posRight = 0; // stores the position of the motor in the previous loop -- position in degrees
int target = 0; // target position -- position in degrees
double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
double current_posLeft = 0; // current position of the motor -- position in degrees
double current_posRight = 0; // current position of the motor -- position in degrees
int Tminus1 = 0;
int T = 0;
// Proportional controller constants
const double Ki = 0.0254;
const double Kp = 0.00152;
int wait = 0; // tracks the current time to determine when the 1 second wait is up 
int beginTime = 0; // the time when the void setup ends
bool start = 1; // ensures that the motors are only set to step once
double velocitySetPoint = 0; // this is the desired velocity of the robot (forward)
double currentPosLeft = 0; //left motor position in degrees
double currentPosRight = 0;//right motor position in degrees
double previousPosLeft = 0;//previous left motor position in degrees
double previousPosRight = 0;//previous right motor position in degrees
double currentVelocity = 0; //current velocity of the robot
int rWheel = 74; // radius of robot wheel in mm



void setup() {

  // Encoder pins
  pinMode(LEFT_PIN_A, INPUT);
  pinMode(LEFT_PIN_B, INPUT);
  pinMode(RIGHT_PIN_A, INPUT);
  pinMode(RIGHT_PIN_B, INPUT);

  Serial.begin(115200);
  
  // Setting up the interrupt handler 
  // Activates whenever the CLK pin on the encoder changes voltage
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN_A), LeftTick, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN_A), RightTick, CHANGE);
  md.init(); //initialize  motor
  delay(500); //delay to let motor initialize
  wait = millis();
  beginTime = wait;
}

void loop() {

  if(start && ((wait-beginTime) >995)){ //do nothing for 1 second then start the step response
    velocitySetPoint = 1;
    start = 0;
    Tminus1 = wait;
  }
  else if (start)wait = millis();
  
  // implement the PI controller
  currentPosLeft = double(countLeft)/4.44;//2.22
  currentPosRight = double(countRight)/4.44;//2.22
  T = millis();
  currentVelocity = double(rWheel)*((currentPosLeft - previousPosLeft)/(T-Tminus1) + (currentPosRight - previousPosRight)/(T-Tminus1))/2;
  currentVelocity = currentVelocity/100; //fix some unit issue I cannot find
  intError = (velocitySetPoint-currentVelocity)*(T-Tminus1);
  motorVal = 1954*((intError*Ki)+((velocitySetPoint-currentVelocity)*Kp));
  md.setM1Speed(motorVal);
  md.setM2Speed(motorVal);
  currMyTime = millis() - beginTime;

  //print out the data
  Serial.print(currMyTime);
  Serial.print(", ");
  Serial.print(currentVelocity*4);
  Serial.println();
 
  delay(5);
  
  if (millis() >=3000+wait){ //Stop the response after 3 seconds
    md.setM1Speed(0);
    md.setM2Speed(0);
    while(1);
  }
  //update the previous values
  Tminus1=T;
  previousPosLeft = currentPosLeft;
  previousPosRight = currentPosRight;
}



// Encoder interrupt
void LeftTick() {
  
  A_CURRENT_STATE_LEFT = digitalRead(LEFT_PIN_A);//read state of signal A
  B_CURRENT_STATE_LEFT = digitalRead(LEFT_PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE_LEFT==B_CURRENT_STATE_LEFT){ //if the signals are the same
    countLeft-=1;//subract from the count
  }
  else {//if signals are different
    countLeft+=1;//add 2 to the count
  }
}

void RightTick() {
  
  A_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_A); //read state of signal A
  B_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_B); //read state of signal B
  
  if (A_CURRENT_STATE_RIGHT==B_CURRENT_STATE_RIGHT){ //if the signals are the same
    countRight+=1;//subract from the count
  }
  else {//if signals are different
    countRight-=1;//add 2 to the count
  }
}
