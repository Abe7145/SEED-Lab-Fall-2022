#include <DualMC33926MotorShield.h>
#include <Wire.h>
#include <PID_v1.h>

#define SLAVE_ADDRESS 0x02
#define PI_ADDRESS 0x20
#define MICROS_TO_SECONDS .000001
#define DEG_TO_RAD 0.017453293
#define MM_TO_CM 0.1

#define START 0
#define THIRTY_DEG_ONE 1
#define THIRTY_DEG_TWO 2
#define ROUGH 3
#define PRECISION 4
#define FIRST_STOP 5
#define GET_POSITION_ONE 6
#define GET_POSITION_TWO 7
#define MOVE_FORWARD 8

#define MOTORVOLTAGE  400
#define FEET_TO_CM 30.48
#define MILLIS_TO_SECONDS .001
///////////////////////////////////////////////////////////////////////////////////////
//\\\\\\\ MOTOR 1 == RIGHT MOTOR   \\\\\\\\\\  MOTOR 2 == LEFT MOTOR \\\\\\\\\\\\\\\//
/////////////////////////////////////////////////////////////////////////////////////////
DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
const int LEFT_PIN_A = 3; 
const int LEFT_PIN_B = 11;
const int RIGHT_PIN_A = 2;
const int RIGHT_PIN_B = 5;
bool request = 0;
int super_secret_code = 0;
double big_boy_error = 0;

int Curr_State = 0;
int Next_State = 0;

// This program uses a state machine to track the encoder’s position
// These variables store the state
bool A_LAST_STATE = 0;
bool A_CURRENT_STATE_LEFT = 0;
bool B_CURRENT_STATE_LEFT = 0;
bool A_CURRENT_STATE_RIGHT = 0;
bool B_CURRENT_STATE_RIGHT = 0;
int key = 1;
int index = 0;
double target_position;

char char_to_send = '0';
bool reading = 0;

//These Varialbes calculate the time between encoder counts
long leftMotorDT = 0;
long leftMotorPrevTime = 0;
long rightMotorDT = 0;
long rightMotorPrevTime = 0;
int leftCountAccumulate_f = 0;
int rightCountAccumulate_f = 0;
int leftCountAccumulate_b = 0;
int rightCountAccumulate_b = 0;
double count_error = 0;  

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
int data;


// Proportional controller constants
const double Ki = .33;//.33
const double Kp = 4.5;//3

//C code for Arduino
//#get the value that the pi sent to determine which quadrent(only 4 cases) it is in
//void receiveData(int byteCount) {
//  case = wire.read(); #variable case is what quadrent or which of the 4 cases the aruco marker is at
//}
void AUSTIN_CALCULATION();
void SCOTT_CALCULATION();
void GLOBAL_CALCULATION();
void MOTOR_MOVE();
void RESET();
void LeftTick();
void RightTick();
void RIGHT_MOTOR_MOVE();
void LEFT_MOTOR_MOVE();
void SCOTT_MOVE();
void SCOTT_RESET();

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


double time_control = 0;

bool RECIEVED_DATA = 0;


double targetInFeet = 0; // target position (feet)
double targetVelocity = 0; //target velocity (cm/s)

double currentPosLeft = 0; // current position of the left motor (degrees)
double currentPosRight = 0; // current position of the right motor (degrees)
double robotPosition = 0; //position of the robot (average of left and right positions) (cm)
double forwardPositionInt = 0; //set position - current position (cm)
int positionErrorTimePrev = 0; //stores time that position error was last updated (milliseconds)
int positionErrorTimeCurr = 0; //stores time at position error update (milliseconds)

double leftVelocityInt = 0; //integral term for left motor velocity (cms)
double rightVelocityInt = 0; //integral term for right motor velocity (cms)///////////////////////////////////////check units here
int velocityLeftErrorTimeCurr = 0; //stores time at left velocity error update (milliseconds)
int velocityRightErrorTimeCurr = 0; //stores time at right velocity error update (milliseconds)
int velocityLeftErrorTimePrev = 0; //stores time that left velocity error was last updated (milliseconds)
int velocityRightErrorTimePrev = 0; //stores time that right velocity error was last updated (milliseconds)
int motorValLeft = 0; //value from -400 to 400 written to the left motor
int motorValRight = 0; //value from -400 to 400 written to the right motor


//controller constants
const double Kp_forwardPos = 0.29867;
const double Ki_forwardPos = 0.0015283;
const double Kp_forwardVelLeft = 0.1233;
const double Kp_forwardVelRight = 0.7105;
const double Ki_forwardVelLeft = 4.9861;
const double Ki_forwardVelRight = 5.8075;


int beginTime = 0; // the time when the void setup ends
bool start = 1; // ensures that the motors are only set to step once

int Tminus1 = 0;
int T = 0;

int prevMyTime = 0; //previous loop time marker -- milliseconds 
int currMyTime = 0; //current time marker -- milliseconds 


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
  Wire.begin(PI_ADDRESS);
  Wire.onReceive(PI_READ);
  Wire.onRequest(PI_WRITE);
}

void loop() {
  GLOBAL_CALCULATION();
  if ( Curr_State != 6 && Curr_State != 7 ) {
      AUSTIN_CALCULATION();
      char_to_send = '0';
  }

  theta_r = theta;
  theta_l = theta;

//  Curr_State = 8;
//  targetInFeet = 4;
  switch ( Curr_State ) {
    case START:
      RESET();
      TOTAL_RESET();
      SCOTT_RESET();
      target = 0;
      super_secret_code = 0;
      Next_State = THIRTY_DEG_ONE;
      break;
      
    case THIRTY_DEG_ONE:
      time_control = millis();
      RESET();
      target += 30;
      Next_State = THIRTY_DEG_TWO;
      
      if (RECIEVED_DATA) {
        Next_State = ROUGH;
        RESET();
        TOTAL_RESET();
        time_control = millis();
        if (data >= 128 ) {
          target = 256 - data;
        }
        else {
          target = -data;
        }
      }
      
      break;

    case THIRTY_DEG_TWO:
      if ( abs(time_control - millis()) >= 2000 ) {
        Next_State = THIRTY_DEG_ONE;
      }

      RIGHT_MOTOR_MOVE();
      LEFT_MOTOR_MOVE();

      if (RECIEVED_DATA) {
        Next_State = ROUGH;
        RESET();
        TOTAL_RESET();
        time_control = millis();
        if (data >= 100 ) {
          target = (256 - data);
        }
        else {
          target = data;
        }
      }
      
      break;
      
    case ROUGH:
      if ( abs(time_control - millis() ) >= 3000 ) {
        Next_State = PRECISION;
        RESET();
        theta = 0;
      }
      RIGHT_MOTOR_MOVE();
      LEFT_MOTOR_MOVE();
      break;
      
    case PRECISION:
      if ( (target) <= 0 ) {
        md.setM1Speed ( 0 );
        md.setM2Speed ( 80 );
      }
      else {
        md.setM1Speed ( 80 );
        md.setM2Speed ( 0 );
      }

      if ( abs(theta) <= 1 ) {
        md.setM2Speed ( 0 );
        md.setM1Speed ( 0 );
        RESET();
        TOTAL_RESET();
        Next_State = FIRST_STOP;
      }
      break;
      
    case FIRST_STOP:
    
      super_secret_code = 123;
      
      if ( request ) {

        reading = 0;
        Next_State = GET_POSITION_ONE;
        
      }
      
      break;
    case GET_POSITION_ONE:
    
      RESET();
      TOTAL_RESET();
      SCOTT_RESET();

      if ( reading == 1) {
        
        Next_State = GET_POSITION_TWO;
        
      }
    
      break;
      
    case GET_POSITION_TWO:
      request = 0;
      targetInFeet = data;
      targetInFeet = double(targetInFeet / 10);
      Next_State = MOVE_FORWARD;
      break;
      
    case MOVE_FORWARD:
    
      SCOTT_MOVE();

      if ( big_boy_error <= 20 ) {
        Next_State = START;
      }
      break;
  
    default:
      break;
  }

  Serial.print('\n');
  Serial.print(theta);
  Serial.print('\t');
  Serial.print(target);
  Serial.print('\t');
  Serial.print(data);
  Serial.print('\t');
  Serial.print(data);
  Curr_State = Next_State;
  
}


// i2c interrupt handler for receive request from PI
void PI_READ(int byteCount) { 

  // stores data sent from Pi to Arduino -- rotation position given by aruco marker

  data = Wire.read(); // receive a byte as character
  RECIEVED_DATA = 1;
  reading = 1;

//  Serial.print("cum");
  
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

void AUSTIN_CALCULATION(){


  
  // if statement is responsible for adhering to a sampling interval. The
  //>= Sampling_Interval means sampling intervals of Sampling_Interval
  // milliseconds
  current_posLeft = double(countLeft)/4.44;//degrees the left wheel has moved
  current_posRight = double(countRight)/4.44;//degrees the right wheel has moved  leftMotorVelocity = (leftMotorAngularVelocity*DEG_TO_RAD)*double(rWheel)*MM_TO_CM; //left motor velocity in cm/sec
  rightMotorVelocity = (rightMotorAngularVelocity*DEG_TO_RAD)*double(rWheel)*MM_TO_CM; //right motor velocity in cm/sec
  x_pos = (current_posLeft + current_posRight)*DEG_TO_RAD*rWheel/2; //calculate the forward distance the robot has moved
  
  if ( (millis() - time_initial) >= Sampling_Interval) {

    time_initial = millis();
    
    theta += (Sampling_Interval / 1000) * (radius / b) * ( leftMotorAngularVelocity - rightMotorAngularVelocity);

  }

}

void GLOBAL_CALCULATION(){

  if ( micros() - leftMotorPrevTime <= 200000 ) {
    if(leftCountAccumulate_f >= 10){
      leftCountAccumulate_f = 0;
      leftCountAccumulate_b = 0;
      leftMotorDT = micros()-leftMotorPrevTime;
      leftMotorPrevTime = micros();
      leftMotorAngularVelocity = (-10.0/4.44)/(leftMotorDT*MICROS_TO_SECONDS);
    }
    if(leftCountAccumulate_b >= 10){
      leftCountAccumulate_f = 0;
      leftCountAccumulate_b = 0;
      leftMotorDT = micros()-leftMotorPrevTime;
      leftMotorPrevTime = micros();
      leftMotorAngularVelocity = (10.0/4.44)/(leftMotorDT*MICROS_TO_SECONDS);
    }
  }
  else {
    leftMotorAngularVelocity = 0;
    leftMotorPrevTime = micros();
    leftCountAccumulate_f = 0;
    leftCountAccumulate_b = 0;
  }
  
  if ( micros() - rightMotorPrevTime <= 200000 ) {
    if(rightCountAccumulate_f >= 10){
      rightCountAccumulate_f = 0;
      rightCountAccumulate_b = 0;
      rightMotorDT = micros()-rightMotorPrevTime;
      rightMotorPrevTime = micros();
      rightMotorAngularVelocity = (10.0/4.44)/(rightMotorDT*MICROS_TO_SECONDS);
    }
    if(rightCountAccumulate_b >= 10){
      rightCountAccumulate_f = 0;
      rightCountAccumulate_b = 0;
      rightMotorDT = micros()-rightMotorPrevTime;
      rightMotorPrevTime = micros();
      rightMotorAngularVelocity = (-10.0/4.44)/(rightMotorDT*MICROS_TO_SECONDS);
    }
  }
  else {
    rightMotorAngularVelocity = 0;
    rightMotorPrevTime = micros();
    rightCountAccumulate_f = 0;
    rightCountAccumulate_b = 0;
  }


  
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

//void RIGHT_MOTOR_MOVE(){
//  Input1 = theta;
//  SetPoint1 = target;
//  pid1.Compute();
//  md.setM1Speed( -Output1);
//}
//
//void LEFT_MOTOR_MOVE(){
//  Input2 = theta;
//  SetPoint2 = target;
//  pid2.Compute();
//  md.setM2Speed( Output2);
//}

void RESET() {

  intError_r = 0;
  intError_l = 0;
  currMyTime_r = millis();
  currMyTime_l = millis();
  RECIEVED_DATA = 0;
  
}

void TOTAL_RESET() {
    theta = 0;
    target = 0;
    rightMotorVelocity = 0;
    left_angular_velocity = 0;
    right_wheel_degrees_initial = 0;
    left_wheel_degrees_initial = 0;
    time_initial = millis();
    theta_l = 0;
    theta_r = 0;
    countLeft = 0;
    countRight = 0;
    rightCountAccumulate_f = 0;
    leftCountAccumulate_b = 0;
    rightCountAccumulate_f = 0;
    leftCountAccumulate_b = 0;
    leftMotorAngularVelocity = 0;
    leftMotorAngularVelocity = 0;
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
//
void PI_WRITE() {

    // Convert current_pos integer into a character array
    // I2C wire write is only compatible with bytes
    // the serial protocol gets confused with integers - must use a char array
    String myNum = String(int(super_secret_code));
    int string_length = myNum.length() + 1;
    char charArray[string_length];
    myNum.toCharArray(charArray, string_length);

    // Send value to Pi
    Wire.write(charArray[index]);
    ++index;
    if (index >= 3) {
         index = 0;
    }

    request = 1;
    
}
//void PI_WRITE() {
//  
//}

void SCOTT_RESET() {

  
  countRight = 0;
  countLeft = 0;

  //calculates the motor velocity in cm/s
  leftMotorVelocity = 0;
  rightMotorVelocity = 0;
  //calculate the position of each wheel
  currentPosLeft = double(countLeft)/4.44;
  currentPosRight = double(countRight)/4.44;

  //calculated the position of the robot
  robotPosition = 0;

  //calculate the forward position error
  positionErrorTimeCurr = millis();
  forwardPositionInt = 0;
  positionErrorTimePrev = positionErrorTimeCurr;
  
  //calculate the velocity error
  velocityLeftErrorTimeCurr = millis();
  velocityRightErrorTimeCurr = millis();
  leftVelocityInt = 0;
  rightVelocityInt = 0;
  //calculate the velocity set values for each motor
  velocityLeftErrorTimePrev = velocityLeftErrorTimeCurr;
  velocityRightErrorTimePrev = velocityRightErrorTimeCurr;

//Need to figure out how to get to voltage, if we want to get to voltage, etc. Check simulation things

  //calculate motor write values
  motorValLeft = 0;
  motorValRight = 0;
  count_error = 0;

  
}

void SCOTT_CALCULATION() {


  T = millis();
  
  //if there are no encoder counts for 100ms, reset the angular velocity to 0
  if(micros() - rightMotorPrevTime >= 100000)rightMotorAngularVelocity = 0;
  if(micros() - leftMotorPrevTime >= 100000)leftMotorAngularVelocity = 0;

  //calculates the motor velocity in cm/s
  leftMotorVelocity = (leftMotorAngularVelocity*DEG_TO_RAD)*double(rWheel)*MM_TO_CM; 
  rightMotorVelocity = (rightMotorAngularVelocity*DEG_TO_RAD)*double(rWheel)*MM_TO_CM;

  //calculate the position of each wheel
  currentPosLeft = double(countLeft)/4.44;
  currentPosRight = double(countRight)/4.44;

  //calculated the position of the robot
  robotPosition = ((currentPosLeft+currentPosRight)*DEG_TO_RAD)*double(rWheel)*MM_TO_CM / 2;


  big_boy_error = targetInFeet*FEET_TO_CM - robotPosition;
  Serial.print('\n');
  Serial.print(big_boy_error);

  if ( big_boy_error >= 120 ) {
    big_boy_error = 120;
  }
  //calculate the forward position error
  positionErrorTimeCurr = millis();
  forwardPositionInt += (big_boy_error)*double((positionErrorTimeCurr-positionErrorTimePrev))*MILLIS_TO_SECONDS;

  // right is motor 1
  count_error = (countRight - countLeft) * double((positionErrorTimeCurr-positionErrorTimePrev))*MILLIS_TO_SECONDS;
  
  positionErrorTimePrev = positionErrorTimeCurr;

  //calculate velocity set point
  targetVelocity = Kp_forwardPos*(big_boy_error)+Ki_forwardPos*forwardPositionInt;
  
  //calculate the velocity error
  velocityLeftErrorTimeCurr = millis();
  velocityRightErrorTimeCurr = millis();
  leftVelocityInt += (targetVelocity - leftMotorVelocity)*double((velocityLeftErrorTimeCurr-velocityLeftErrorTimePrev))*MILLIS_TO_SECONDS;
  rightVelocityInt += (targetVelocity - rightMotorVelocity)*double((velocityRightErrorTimeCurr-velocityRightErrorTimePrev))*MILLIS_TO_SECONDS;
  //calculate the velocity set values for each motor
  leftVelocityInt += (targetVelocity-leftMotorVelocity)*double((velocityLeftErrorTimeCurr-velocityLeftErrorTimePrev))*MILLIS_TO_SECONDS;
  rightVelocityInt += (targetVelocity-rightMotorVelocity)*double((velocityRightErrorTimeCurr-velocityRightErrorTimePrev))*MILLIS_TO_SECONDS;
  velocityLeftErrorTimePrev = velocityLeftErrorTimeCurr;
  velocityRightErrorTimePrev = velocityRightErrorTimeCurr;

//Need to figure out how to get to voltage, if we want to get to voltage, etc. Check simulation things

  //calculate motor write values
  motorValLeft = Kp_forwardVelLeft*(targetVelocity-leftMotorVelocity)+Ki_forwardVelLeft*(leftVelocityInt);
  motorValRight = Kp_forwardVelRight*(targetVelocity-rightMotorVelocity)+Ki_forwardVelRight*(rightVelocityInt);
//
//  if ( motorValLeft >= 250 ) {
//    motorValLeft = 250;
//  }
//  if ( motorValRight >= 250 ) {
//    motorValRight = 250;
//  }

  // right is motor 1
  count_error = countRight - countLeft;

//  motorValRight -= count_error * .4;

  Tminus1=T;

}

void SCOTT_MOVE(){

  SCOTT_CALCULATION();
  
  md.setM1Speed(motorValRight);
  md.setM2Speed(motorValLeft);
  currMyTime = millis() - beginTime; // remove program startup time from time measurement variable
  //Print out the necessary values
  //Serial.print(currMyTime);
//  Serial.print(", ");
//  Serial.print(currMyTime);
//  Serial.print(", ");
//  Serial.print(motorValLeft);
//  Serial.print(", ");
//  Serial.print(motorValRight);
//  Serial.print(", ");
//  Serial.print(targetVelocity);
//  Serial.println();
 
  delay(5);
  
//  if (millis() >=15000+wait){ //stop the response after 3 seconds
//    md.setM1Speed(0);
//    md.setM2Speed(0);
//    while(1);
//    
//  }

}
