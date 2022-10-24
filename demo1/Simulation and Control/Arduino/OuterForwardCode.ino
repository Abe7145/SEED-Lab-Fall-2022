/*
 * The Seedsters
 * Created by Scott Reeder
 * 
 * This code implemments both inner and outer PI conroller loops. It takes a position and
 * moves the robot forward to the desired position.
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
int outerIntError = 0;    
int prevMyTime = 0; //previous loop time marker -- milliseconds--only used for printing time stamp of data
int currMyTime = 0; //current time marker -- millisecond--only used for printing time stamp of data
double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
int Tminus1 = 0;
int T = 0;
// Proportional controller constants
const double Ki = 0.0254;//inner integral term
const double Kp = 0.00152;//inner p term
const double Kiouter = 0.0019;//0.00223;//outer integral term
const double Kpouter = 0.325;//0.3576;//outer p term
int wait = 0; //variable used to tell when 1 second wait has elapsed for step responses
int beginTime = 0;//notes how long the processor startup time is
bool start = 1; //used to make the setpoint be set only once
double positionSetPoint = 0; // this is the desired position of the robot (forward) in mm
double velocitySetPoint = 0; // velocity that the motor should be at
int currentPosition = 0; //current net position rWheel*(currentPosLeft+currentPosRight)/2 in mm
double currentPosLeft = 0; //current position of the left wheel in degrees
double currentPosRight = 0; //current position of the right wheel in degrees
double previousPosLeft = 0;
double previousPosRight = 0;
double currentVelocity = 0;
int rWheel = 74; // in mm
double degToRad = 0.01745; //rad/deg
double newSet = 0;
//C code for Arduino
//#get the value that the pi sent to determine which quadrent(only 4 cases) it is in
//void receiveData(int byteCount) {
//  case = wire.read(); #variable case is what quadrent or which of the 4 cases the aruco marker is at
//}

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
    positionSetPoint = 1000;
    start = 0;
    Tminus1 = wait;
  }
  else if (start)wait = millis();

  //implement te pi controller
  currentPosLeft = double(countLeft)/4.44;//2.22
  currentPosRight = double(countRight)/4.44;//2.22
  T = millis();
  currentPosition = double(rWheel)*degToRad*(currentPosLeft+currentPosRight)/2; //simulink out.simout val
  outerIntError = (positionSetPoint-currentPosition)*(T-Tminus1); //signal to outer pi controller integral term
  velocitySetPoint = (positionSetPoint-currentPosition)*Kpouter + outerIntError*Kiouter; // signal to inner sum block
  currentVelocity = double(rWheel)*((currentPosLeft - previousPosLeft)/(T-Tminus1) + (currentPosRight - previousPosRight)/(T-Tminus1))/2;//be careful of divide by 0
  intError = (velocitySetPoint-currentVelocity)*(T-Tminus1);
  motorVal = 20*((intError*Ki)+((velocitySetPoint-currentVelocity)*Kp));//signal going to motor Multiply by 20 for 8V/400 counts *1000 for using ms = 20
  //write the motor values
  md.setM1Speed(motorVal);
  md.setM2Speed(motorVal*.9);//.9 to account for motor differences
  currMyTime = millis() - beginTime;
  Serial.print(currMyTime);//currMyTime
  Serial.print(", ");
  Serial.print(currentPosition);
  Serial.println();
 
  delay(5);

  //update the memory variables
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
