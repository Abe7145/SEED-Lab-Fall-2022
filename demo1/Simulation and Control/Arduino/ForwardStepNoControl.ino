/*
 * The Seedsters
 * Created by Scott Reeder
 * 
 * This code implements a step response for forward motion of the robot. It then prints out time
 * and velocity values to the serial monitor
 */

#include <DualMC33926MotorShield.h>


//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//\\\\\\\ MOTOR 1 == RIGHT MOTOR   \\\\\\\\\\  MOTOR 2 == LEFT MOTOR \\\\\\\\\\\\\\\\\
/////////////////////////////////////////////////////////////////////////////////////
DualMC33926MotorShield md;//create motor object

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
#define MOTORVOLTAGE  400
const int LEFT_PIN_A = 3; //encoder 1 pin A
const int LEFT_PIN_B = 11; //encoder 1 pin B
const int RIGHT_PIN_A = 2; //encoder 2 pin A
const int RIGHT_PIN_B = 5; //encoder 2 pin B

// This program uses a state machine to track the encoder’s position
// These variables store the state of the encoder
bool A_CURRENT_STATE_LEFT = 0;
bool B_CURRENT_STATE_LEFT = 0;
bool A_CURRENT_STATE_RIGHT = 0;
bool B_CURRENT_STATE_RIGHT = 0;

int index = 0;

int countLeft; // measures left motor encoder counts -- counts
int countRight; // measures right motor encoder counts -- counts
int intError = 0; //integral error for I in PI controller       
int prevMyTime = 0; //previous loop time marker -- milliseconds 
int currMyTime = 0; //current time marker -- milliseconds 
int prev_pos = 0; // stores the position of the motor in the previous loop -- position in degrees
int target = 0; // target position -- position in degrees
double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
double current_posLeft = 0; // current position of the motor -- position in degrees
double current_posRight = 0; // current position of the motor -- position in degrees

// Proportional controller constants

int wait = 0; // tracks the current time to determine when the 1 second wait is up 
int beginTime = 0; // the time when the void setup ends
bool start = 1; // ensures that the motors are only set to step once

void setup() {

  // Intialize Encoder pins
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
  wait = millis(); //get the current program time
  beginTime = wait; //store the program time as the begining of the void loop
}

void loop() {

  // Normalizing the unitless encoder counts into a 360 degree rotation
  current_posLeft = double(countLeft)/4.44;//2.22
  current_posRight = double(countRight)/4.44;//2.22

  if(start && ((wait-beginTime) >995)){ //when 1 second has elapsed, begin the step response
    //set motors to final step value
    md.setM1Speed(MOTORVOLTAGE/2);
    md.setM2Speed(MOTORVOLTAGE/2);
    start = 0;// note that motors have already been set
  }
  else if (start)wait = millis();
  
  currMyTime = millis() - beginTime; // remove program startup time from time measurement variable
 
  //Print out the necessary values
  Serial.print(currMyTime);
  Serial.print(", ");
  Serial.print(current_posLeft);
  Serial.print(", ");
  Serial.print(current_posRight);
  Serial.println();
 
  delay(5);
  
  if (millis() >=3000+wait){ //stop the response after 3 seconds
    md.setM1Speed(0);
    md.setM2Speed(0);
    while(1);
  }

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
