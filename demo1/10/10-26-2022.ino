#include <DualMC33926MotorShield.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x02
#define PI_ADDRESS 0x20

#define MICROS_TO_SECONDS .000001
#define DEG_TO_RAD 0.017453293
#define MM_TO_CM 0.1
///////////////////////////////////////////////////////////////////////////////////////
//\\\\\\\ MOTOR 1 == RIGHT MOTOR   \\\\\\\\\\  MOTOR 2 == LEFT MOTOR \\\\\\\\\\\\\\\//
/////////////////////////////////////////////////////////////////////////////////////////
DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
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
int key = 1;
int index = 0;

//These Varialbes calculate the time between encoder counts
long leftMotorDT = 0;
long leftMotorPrevTime = 0;
long rightMotorDT = 0;
long rightMotorPrevTime = 0;
int leftCountAccumulate_f = 0;
int rightCountAccumulate_f = 0;
int leftCountAccumulate_b = 0;
int rightCountAccumulate_b = 0;

//These variables store the discrete instantaneous angular velocity
double leftMotorAngularVelocity = 0;//in 
double rightMotorAngularVelocity = 0;

double current_posLeft = 0; // current position of the motor -- position in degrees
double current_posRight = 0; // current position of the motor -- position in degrees
double countLeft = 0;
double countRight = 0;

double target = 0;
double MOVE_FEET = 7;

double theta_r = 0;
double intError_r = 0;
double currMyTime_r = 0;
double prevMyTime_r = 0;
double prev_pos_r = 0;
double motorVal_r = 0;

double theta_l = 0;
double intError_l = 0;
double currMyTime_l = 0;
double prevMyTime_l = 0;
double prev_pos_l = 0;
double motorVal_l = 0;

double time_initial = 0;
double x_pos = 0;
double y_pos = 0;
double theta = 0;
double right_wheel_degrees_initial = 0;
double left_wheel_degrees_initial = 0;
double rightMotorVelocity = 0;
double left_angular_velocity = 0;
double radius = 7.4;
double b = 31;
const double Sampling_Interval= 10;
int leftTnow = 0;
int leftTprev = 0;
int rightTnow = 0;
int rightTprev = 0;


double vel_error;
double wait;


// Proportional controller constants
const double Ki = .13;//.33
const double Kp = 4.5;//3

//C code for Arduino
//#get the value that the pi sent to determine which quadrent(only 4 cases) it is in
//void receiveData(int byteCount) {
//  case = wire.read(); #variable case is what quadrent or which of the 4 cases the aruco marker is at
//}
void CALCULATIONS();
void MOTOR_MOVE();

double m1_velocity = 0;
double m2_velocity = 0;
double m1_velocity_error = 0;
double m2_velocity_error = 0;
double time_initial_rotate = 0;
double iterate = 0;
double ki_move_forward = 5.3;//5.3,6,5.5
double kp_move_forward = 0;//1
double speed1;
double speed2;
double speed_error_left = 0;
double speed_error_right = 0;
double angle_multiplier = 0;

int rWheel = 74; // radius of robot wheel in mm
double leftMotorVelocity =0;

void setup() {

  // Encoder pins
  pinMode(LEFT_PIN_A, INPUT);
  pinMode(LEFT_PIN_B, INPUT);
  pinMode(RIGHT_PIN_A, INPUT);
  pinMode(RIGHT_PIN_B, INPUT);

  // i2c pins
  pinMode(12,INPUT);
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  
  // Setting up the interrupt handler 
  // Activates whenever the CLK pin on the encoder changes voltage
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN_A), LeftTick, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN_A), RightTick, CHANGE);
  md.init(); //initialize  motor
  delay(500); //delay to let motor initialize
  
}

void loop() {

  // Nothing happens in the main loop due to interrupt handler
  // Serial.println("Austin is a dumbass");

  theta_r = theta;
  theta_l = theta;

  // Normalizing the unitless encoder counts into a 360 degree rotation
  current_posLeft = double(countLeft)/4.44;//2.22
  current_posRight = double(countRight)/4.44;//2.22

  CALCULATIONS();

  if ( key != 3 ) {
    RIGHT_MOTOR_MOVE();
    LEFT_MOTOR_MOVE();
  }


  if ( ((target - theta <= 10) ||(target-theta<=-10))&& (key == 1) ) {
    Serial.println("asdf");
    wait = millis();
    key = 2;

  }

  if (( (millis() - wait) >= 2000 ) && key == 2) {
    theta = 0;
    target = 0;
    x_pos = 0;
    y_pos = 0;
    rightMotorVelocity = 0;
    left_angular_velocity = 0;
    right_wheel_degrees_initial = 0;
    left_wheel_degrees_initial = 0;
    time_initial = millis();
    key = 3;
    theta_l = 0;
    theta_r = 0;
    countLeft = 0;
    countRight = 0;
  }

  if (key == 3 ) {
    Serial.println("asdf");
    MOTOR_MOVE();
  }
  
  Serial.print('\n');
  Serial.print(rightMotorAngularVelocity);
  Serial.print('\t');
  Serial.print(speed_error_right - speed_error_left);
  Serial.print('\t');
  Serial.print(x_pos);
  Serial.print('\t');
  Serial.print(theta);
  Serial.print('\t');
  Serial.print(MOVE_FEET - x_pos);
 
}


// i2c interrupt handler for receive request from PI
void PI_READ(int byteCount) { 

  // stores data sent from Pi to Arduino -- rotation position given by aruco marker
  int data = 0;
  //data = Wire.read(); // receive a byte as character
  

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

// instantiate index to iterate through charArray inside of the PI_WRITE loop


// i2c interrupt handler for request request from PI


// Encoder interrupt
void LeftTick() {


  A_CURRENT_STATE_LEFT = digitalRead(LEFT_PIN_A);//read state of signal A
  B_CURRENT_STATE_LEFT = digitalRead(LEFT_PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE_LEFT==B_CURRENT_STATE_LEFT){ //if the signals are the same

    countLeft-=1;//subract from the count
    leftCountAccumulate_f ++;
  }
  else {//if signals are different
    leftCountAccumulate_b ++;
    countLeft+=1;//add 2 to the count
  }
}

void RightTick() {

  A_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_A); //read state of signal A
  B_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_B); //read state of signal B
  
  if (A_CURRENT_STATE_RIGHT==B_CURRENT_STATE_RIGHT){ //if the signals are the same
    rightCountAccumulate_f ++;
    countRight+=1;//subract from the count
  }
  else {//if signals are different
    rightCountAccumulate_b ++;
    countRight-=1;//add 2 to the count
  }
}

void CALCULATIONS(){

  if ( micros() - leftMotorPrevTime <= 100000 ) {
    if(leftCountAccumulate_f >= 10){
      leftCountAccumulate_f = 0;
      leftMotorDT = micros()-leftMotorPrevTime;
      leftMotorPrevTime = micros();
      leftMotorAngularVelocity = (10.0/4.44)/(leftMotorDT*MICROS_TO_SECONDS);
    }
    if(leftCountAccumulate_b >= 10){
      leftCountAccumulate_b = 0;
      leftMotorDT = micros()-leftMotorPrevTime;
      leftMotorPrevTime = micros();
      leftMotorAngularVelocity = (-10.0/4.44)/(leftMotorDT*MICROS_TO_SECONDS);
    }
  }
  else {
    leftMotorAngularVelocity = 0;
    leftMotorPrevTime = micros();
  }
  
  if ( micros() - rightMotorPrevTime <= 100000 ) {
    if(rightCountAccumulate_f >= 10){
      rightCountAccumulate_f = 0;
      rightMotorDT = micros()-rightMotorPrevTime;
      rightMotorPrevTime = micros();
      rightMotorAngularVelocity = (10.0/4.44)/(rightMotorDT*MICROS_TO_SECONDS); 
    }
    if(rightCountAccumulate_b >= 10){
      rightCountAccumulate_b = 0;
      rightMotorDT = micros()-rightMotorPrevTime;
      rightMotorPrevTime = micros();
      rightMotorAngularVelocity = (-10.0/4.44)/(rightMotorDT*MICROS_TO_SECONDS); 
    }
  }
  else {
    rightMotorAngularVelocity = 0;
    rightMotorPrevTime = micros();
  }


  
  // if statement is responsible for adhering to a sampling interval. The
  //>= Sampling_Interval means sampling intervals of Sampling_Interval
  // milliseconds
  current_posLeft = double(countLeft)/4.44;//degrees the left wheel has moved
  current_posRight = double(countRight)/4.44;//degrees the right wheel has moved  leftMotorVelocity = (leftMotorAngularVelocity*DEG_TO_RAD)*double(rWheel)*MM_TO_CM; //left motor velocity in cm/sec
  rightMotorVelocity = (rightMotorAngularVelocity*DEG_TO_RAD)*double(rWheel)*MM_TO_CM; //right motor velocity in cm/sec
  x_pos = (current_posLeft + current_posRight)*DEG_TO_RAD*rWheel/2; //calculate the forward distance the robot has moved
  
  if ( (millis() - time_initial) >= Sampling_Interval) {

    time_initial = millis();
    
    // Tracking position of car
    //y_pos += (Sampling_Interval / 1000) * sin( theta ) * (rightMotorVelocity + left_angular_velocity) / 2;
    theta += (Sampling_Interval / 1000) * (radius / b) * -( leftMotorAngularVelocity - rightMotorAngularVelocity);

  }
//  Serial.print(double(x_pos));
//  Serial.print('\n');

}




void RIGHT_MOTOR_MOVE(){
  
  if (theta_r-target <=  3 && theta_r-target >=  -3){// THIS IF STATEMENT IS SUPER DUPER IMPORTANT!!!!!!!!!!!!
    theta_r = target;                // There is some noise in the system that will cause everything to become unstable
    intError_r = 0;                         // This if statement rejects such noise and keeps the system from running away
  }
  
  currMyTime_r = millis();//get current time
  if (currMyTime_r >= prevMyTime_r+100){ // sample every 10th of a second

    // calculating integral error
    if ( abs((target - theta_r )) < 50 ) {
       intError_r += (target- theta_r);
    }
    else {
      intError_r = 0;
    }
    // storing dt
    prev_pos_r = theta_r;//reset prev_pos
    prevMyTime_r = currMyTime_r;//reset prevMyTime

    // use built in shield library to move the motor 
    // takes in values from -400 to 400
    // uses a built in PWM to control the voltage supplied to the motor
    motorVal_r = double(( target-theta_r )) * Kp + (Ki * double(intError_r)); //calculate the motor val

    if ( motorVal_r >= 150 ) {
      motorVal_r = 150;
    }
    md.setM1Speed( -motorVal_r);
    
  }
  
}

void LEFT_MOTOR_MOVE(){

  
  if (theta_l-target <=  3 && theta_l-target >=  -3){// THIS IF STATEMENT IS SUPER DUPER IMPORTANT!!!!!!!!!!!!
    theta_l = target;                // There is some noise in the system that will cause everything to become unstable
    intError_r = 0;                         // This if statement rejects such noise and keeps the system from running away
  }
  
  currMyTime_l = millis();//get current time
  if (currMyTime_l >= prevMyTime_l+100){ // sample every 10th of a second

    // calculating integral error
    if ( abs((target - theta_l )) < 50 ) {
       intError_l += (target- theta_l);
    }
    else {
      intError_l = 0;
    }

    // storing dt
    prev_pos_l = theta_l;//reset prev_pos
    prevMyTime_l = currMyTime_l;//reset prevMyTime

    // use built in shield library to move the motor 
    // takes in values from -400 to 400
    // uses a built in PWM to control the voltage supplied to the motor
    motorVal_l = double(( target-theta_l )) * Kp + (Ki * double(intError_l));

    if ( motorVal_l >= 150 ) {
      motorVal_l = 150;
    }
    md.setM2Speed( motorVal_l);
    
  }
  
}


void MOTOR_MOVE() {
  
  
  // forcing sampling rate to Sampling_Interval
  if ( millis() - time_initial >= Sampling_Interval ) {

    // Tracking time elapsed
    iterate += 1;
    time_initial_rotate = millis();

    // Error term for position
    vel_error = (MOVE_FEET*.965 - x_pos ) * 3;  

    // Saturation 
    if ( vel_error >= 5 ) {

      vel_error = 5;
      
    }

    // Error for controlling angular velocity
    speed_error_right = (vel_error - rightMotorVelocity);
    speed_error_left = (vel_error - leftMotorVelocity);

    // Exaggerate compensation if one wheel is going faster than the other
    if ( (rightMotorVelocity > left_angular_velocity) ) {
      speed_error_left += abs(speed_error_right - speed_error_left) * 3;//3
    }
    else {
      speed_error_right += abs(speed_error_left - speed_error_right) * 3;
    }

    // Noise error
    if ( abs(speed_error_right) <= 1.5) { 
      speed_error_right = 0;
    }

    if ( abs(speed_error_left) <= 1.5 ) { 
      speed_error_left = 0;
    }

    // Calculating integral error 
    m1_velocity += ( speed_error_right ) * Sampling_Interval * iterate / 1000;
    m2_velocity += ( speed_error_left )  * Sampling_Interval * iterate / 1000;

  }

  // PID controller for position movement
  speed1 = (vel_error * kp_move_forward + m1_velocity * ki_move_forward);
  speed2 = (vel_error * kp_move_forward + m2_velocity * ki_move_forward);

  md.setM1Speed( speed1 );

  // Exponential to start the weaker wheel off quicker to match the stronger wheel
  md.setM2Speed( speed2 * exp(1.8/(iterate)));

}
