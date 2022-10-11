#include <DualMC33926MotorShield.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x02
#define PI_ADDRESS 0x20
#define Sampling_Interval 100


DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
const int PIN_A_r = 2;
const int PIN_B_r = 5;

const int PIN_A_l = 3;
const int PIN_B_l = 6;

// This program uses a state machine to track the encoder’s position
// These variables store the state
bool A_LAST_STATE_r = 0;
bool A_CURRENT_STATE_r = 0;
bool B_CURRENT_STATE_r = 0;
bool A_LAST_STATE_l = 0;
bool A_CURRENT_STATE_l = 0;
bool B_CURRENT_STATE_l = 0;


int count_r; // measures motor encoder counts -- unitless
int count_l; // measures motor encoder counts -- unitless

int target = 0; // target position -- position in degrees

int intError_r = 0; //integral error for I in PI controller -- tenths of a degree second       
int prevMyTime_r = 0; //previous loop time marker -- milliseconds 
int currMyTime_r = 0; //current time marker -- milliseconds 
int prev_pos_r = 0; // stores the position of the motor in the previous loop -- position in degrees
double motorVal_r = 0;//variable to hold the value we write to the motor -- units in degrees
double current_pos_r = 0; // current position of the motor -- position in degrees

int intError_l = 0; //integral error for I in PI controller -- tenths of a degree second       
int prevMyTime_l = 0; //previous loop time marker -- milliseconds 
int currMyTime_l = 0; //current time marker -- milliseconds 
int prev_pos_l = 0; // stores the position of the motor in the previous loop -- position in degrees
double motorVal_l = 0;//variable to hold the value we write to the motor -- units in degrees
double current_pos_l = 0; // current position of the motor -- position in degrees

// Proportional controller constants
const double Ki = 0.01;
const double Kp = 3.5;

// variable instantiations
double time_initial;
double x_pos = 0;
double y_pos = 0;
double theta = 0;
int right_wheel_degrees_initial = 0;
int left_wheel_degrees_initial = 0;
double right_angular_velocity = 0;
double left_angular_velocity = 0;

// iteration variable
int i = -1;



void RIGHT_MOTOR_MOVE();
void LEFT_MOTOR_MOVE();
void CALCULATIONS();

void setup() {

  // Encoder pins
  pinMode(PIN_A_r, INPUT);
  pinMode(PIN_B_r, INPUT);

  // i2c pins
  pinMode(12,INPUT);
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  
  // Setting up the interrupt handler 
  // Activates whenever the CLK pin on the encoder changes voltage
  attachInterrupt(digitalPinToInterrupt(2), TICK_RIGHT, CHANGE); \
  attachInterrupt(digitalPinToInterrupt(2), TICK_LEFT, CHANGE); 
  
  md.init(); //intialize motor
  delay(500); //delay to let motor initialize

  // i2c interrupt initialization 
  Wire.begin(PI_ADDRESS);
  Wire.onReceive(PI_READ);
  
}

void loop() {
  
  CALCULATIONS();
  RIGHT_MOTOR_MOVE();
  LEFT_MOTOR_MOVE();

}


// i2c interrupt handler for receive request from PI
void PI_READ(int byteCount) { 

  // stores data sent from Pi to Arduino -- rotation position given by aruco marker
  int data = 0;
  data = Wire.read(); // receive a byte as character
  

  // PI sends 0,1,2,3 depending on quadrant the aruco marker is placed in
  switch(data) {
    case 0:
      target = 0; 
      break;
    case 1:
      target = 90;
      break;
    case 2:
      target = 180;
      break;
    case 3:
      target = 270;
      break;
  }

}

// Encoder interrupt
void TICK_RIGHT() {

  A_CURRENT_STATE_r = digitalRead(PIN_A_r);//read state of signal A
  B_CURRENT_STATE_r = digitalRead(PIN_B_r);//read state of signal B
  
  if (A_CURRENT_STATE_r==B_CURRENT_STATE_r){ //if the signals are the same

    count_r-=1;//subract from the count
  }
  else {//if signals are different
    count_r+=1;//add 2 to the count
  }

}

void TICK_LEFT() {


  A_CURRENT_STATE_l = digitalRead(PIN_A_l);//read state of signal A
  B_CURRENT_STATE_l = digitalRead(PIN_B_l);//read state of signal B
  
  if (A_CURRENT_STATE_l==B_CURRENT_STATE_l){ //if the signals are the same

    count_l-=1;//subract from the count
  }
  else {//if signals are different
    count_l+=1;//add 2 to the count
  }

}



void RIGHT_MOTOR_MOVE(){
  
  // Nothing happens in the main loop due to interrupt handler
  // Serial.println("Austin is a dumbass");

  // Normalizing the unitless encoder counts into a 360 degree rotation
  current_pos_r = double(count_r)/4.44;//2.22

  
  if (theta-target <=  10 && theta-target_ >=  -10){// THIS IF STATEMENT IS SUPER DUPER IMPORTANT!!!!!!!!!!!!
    theta = target;                // There is some noise in the system that will cause everything to become unstable
    intError_r = 0;                         // This if statement rejects such noise and keeps the system from running away
  }
  
  currMyTime_r = millis();//get current time
  if (currMyTime_r >= prevMyTime_r+100){ // sample every 10th of a second

    // calculating integral error
    intError_r += (theta-target);

    // storing dt
    prev_pos_r = theta;//reset prev_pos
    prevMyTime_r = currMyTime_r;//reset prevMyTime

    // use built in shield library to move the motor 
    // takes in values from -400 to 400
    // uses a built in PWM to control the voltage supplied to the motor
    motorVal_r = double(( target_r-theta )) * Kp + (Ki * double(intError_r)); //calculate the motor val
    md.setM1Speed( motorVal_r );
    
  }
  
}

void LEFT_MOTOR_MOVE(){
  
  // Nothing happens in the main loop due to interrupt handler
  // Serial.println("Austin is a dumbass");

  // Normalizing the unitless encoder counts into a 360 degree rotation
  current_pos_l = double(count_l)/4.44;//2.22

  
  if (theta-target <=  10 && theta-target_ >=  -10){// THIS IF STATEMENT IS SUPER DUPER IMPORTANT!!!!!!!!!!!!
    theta = target;                // There is some noise in the system that will cause everything to become unstable
    intError_r = 0;                         // This if statement rejects such noise and keeps the system from running away
  }
  
  currMyTime_l = millis();//get current time
  if (currMyTime_l >= prevMyTime_l+100){ // sample every 10th of a second

    // calculating integral error
    intError_l += (theta-target);

    // storing dt
    prev_pos_l = theta;//reset prev_pos
    prevMyTime_l = currMyTime_l;//reset prevMyTime

    // use built in shield library to move the motor 
    // takes in values from -400 to 400
    // uses a built in PWM to control the voltage supplied to the motor
    motorVal_l = double(( target_l-theta )) * Kp + (Ki * double(intError_l)); //calculate the motor val
    md.setM2Speed( motorVal_l );
    
  }
}

void CALCULATIONS(){
  // if statement is responsible for adhering to a sampling interval. The
  //>= Sampling_Interval means sampling intervals of Sampling_Interval
  // milliseconds
  
  if ( (millis() - time_initial) >= Sampling_Interval) {
    i += 1;
    time_initial = millis();
    
    // Angular velocity equations
    right_angular_velocity = ( 18 * ( current_pos_r - right_wheel_degrees_initial) ) * (3.14159) / 180;
    left_angular_velocity = ( 18 * ( current_pos_l - left_wheel_degrees_initial )) * (3.14159) / 180;
    
    right_wheel_degrees_initial = current_pos_r;
    left_wheel_degrees_initial = current_pos_l;
    
    // Tracking position of car
    x_pos += .5 * cos( theta ) * (right_angular_velocity * radius + left_angular_velocity * radius) / 2;
    y_pos += .5 * sin( theta ) * (right_angular_velocity * radius + left_angular_velocity * radius) / 2;
    theta += .5 * radius / b * ( left_angular_velocity * radius - right_angular_velocity * radius);

  }
}
