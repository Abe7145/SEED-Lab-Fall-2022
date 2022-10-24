/*
 * The Seedsters
 * Created by Scott Reeder
 * 
 * This code implements a step response for rotation of the robot
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
double intError = 0; //integral error for I in PI controller -- tenths of a degree second       
int prevMyTime = 0; //previous loop time marker -- milliseconds 
int currMyTime = 0; //current time marker -- milliseconds 
//int prev_posLeft = 0; // stores the position of the motor in the previous loop -- position in degrees
//int prev_posRight = 0; // stores the position of the motor in the previous loop -- position in degrees
int target = 0; // target position -- position in degrees
double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
double current_posLeft = 0; // current position of the motor -- position in degrees
double current_posRight = 0; // current position of the motor -- position in degrees
//time tracking variables
int Tminus1 = 0;
int T = 0;
// Proportional controller constants
const double Ki = 4.004;
const double Kp = 0.53738;
int wait = 0;
int beginTime = 0;
bool start = 1;
double velocitySetPoint = 0; // this is the desired velocity of the robot (forward)
double currentPosLeft = 0; //current position fo the left motor
double currentPosRight = 0; //current position for the right motor
double previousPosLeft = 0; //previous position for the left motor
double previousPosRight = 0; //previous position for the right motor
double currentVelocity = 0; //angular velocity of the robot
int rWheel = 74; // robot wheel radius in mm
int dRobot = 173;

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
  
  if(start && ((wait-beginTime) >995)){
    velocitySetPoint = 1;
    start = 0;
    Tminus1 = wait;
  }
  else if (start)wait = millis();
  //implement PI controller
  currentPosLeft = double(countLeft)/4.44;//2.22
  currentPosRight = double(countRight)/4.44;//2.22
  T = millis();
  currentVelocity = double(rWheel)*((currentPosRight - previousPosRight)/(T-Tminus1)-(currentPosLeft - previousPosLeft)/(T-Tminus1))/double(dRobot);//be careful of divide by 0
  currentVelocity = currentVelocity; //fix some unit issue I cannot find
  intError = (velocitySetPoint-currentVelocity)*(T-Tminus1);
  motorVal = 13*((intError*Ki)+((velocitySetPoint-currentVelocity)*Kp));
  md.setM1Speed(motorVal);
  md.setM2Speed(-motorVal*.9);
  currMyTime = millis() - beginTime;
  Serial.print(currMyTime);
  Serial.print(", ");
  Serial.print(currentVelocity*4.2);
  Serial.println();
 
  delay(5);
  
  if (millis() >=3000+wait){//stop the step response
    md.setM1Speed(0);
    md.setM2Speed(0);
    while(1);
  }

  //update memory variables
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
