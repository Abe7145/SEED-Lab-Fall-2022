/*
 * The Seedsters
 * Created by Scott Reeder
 * 
 * This code implements both angular velocity and position controllers
 * 
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
double outerIntError = 0;    
int prevMyTime = 0; //previous loop time marker -- milliseconds--only used for printing time stamp of data
int currMyTime = 0; //current time marker -- millisecond--only used for printing time stamp of data
double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
int Tminus1 = 0; //time of previous loop 
int T = 0; //time of currnet loop

// PI controller constants
const double Ki = 4.004;//inner integral term
const double Kp = 0.53738;//inner p term
const double Kiouter = 1.272;//outer integral term
const double Kpouter = 8.506;//outer p term

int wait = 0; //variable used to tell when 1 second wait has elapsed for step responses
int beginTime = 0;//notes how long the processor startup time is
bool start = 1; //used to make the setpoint be set only once
double angleSetPoint = 0; // this is the desired position of the robot (forward) in mm
double velocitySetPoint = 0; // velocity that the motor should be at
int currentPosition = 0; //current net position rWheel*(currentPosLeft+currentPosRight)/2 in mm
double currentPosLeft = 0; //current position of the left wheel in degrees
double currentPosRight = 0; //current position of the right wheel in degrees
double previousPosLeft = 0; //previous position of the left wheel in degrees
double previousPosRight = 0; //previous position of the right wheel in degrees
double currentVelocity = 0; //angular velocity of robot (in rad/sec)
int rWheel = 74; // radius of wheel in mm
//double newSet = 0;
int dRobot = 173;//diameter of the robot in mm


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
    angleSetPoint = 172;
    start = 0;
    Tminus1 = wait;
  }
  else if (start)wait = millis();
  //implement the PI controllers
  currentPosLeft = double(countLeft)/4.44;//2.22
  currentPosRight = double(countRight)/4.44;//2.22
  T = millis();
  currentPosition = double(rWheel)*(currentPosRight-currentPosLeft)/(2*dRobot); //simulink out.simout val
  outerIntError = (angleSetPoint-currentPosition)*(T-Tminus1)/1000; //signal to outer pi controller integral term
  velocitySetPoint = (angleSetPoint-currentPosition)*Kpouter + outerIntError*Kiouter; // signal to inner sum block
  currentVelocity = double(rWheel)*((currentPosRight - previousPosRight)/(T-Tminus1)-(currentPosLeft - previousPosLeft)/(T-Tminus1))/double(dRobot);//be careful of divide by 0
  intError = (velocitySetPoint-currentVelocity)*(T-Tminus1)/1000;
  motorVal = ((intError*Ki)+((velocitySetPoint-currentVelocity)*Kp));//signal going to motor
  //write the motor values
  md.setM1Speed(motorVal);
  md.setM2Speed(-motorVal*.9);
  currMyTime = millis() - beginTime;
  Serial.print(currMyTime);
  Serial.print(", ");
  Serial.print(currentPosition);
  Serial.println();
 
  delay(10);

  //update the memory values
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
    countLeft+=1;//add 1 to the count
  }
}

void RightTick() { //encoder count increments are inverted because of motor mounting. This way positive motor counts describe forward robot movement

  A_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_A); //read state of signal A
  B_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_B); //read state of signal B
  
  if (A_CURRENT_STATE_RIGHT==B_CURRENT_STATE_RIGHT){ //if the signals are the same

    countRight+=1;//add 1 to the count
  }
  else {//if signals are different
    countRight-=1;//subract from the count
  }
}
