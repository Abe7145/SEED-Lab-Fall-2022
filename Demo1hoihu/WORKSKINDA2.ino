#include <DualMC33926MotorShield.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x02
#define PI_ADDRESS 0x20

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

//int countLeft; // measures motor encoder counts -- unitless
//int countRight;
//int intError = 0; //integral error for I in PI controller -- tenths of a degree second       
//int prevMyTime = 0; //previous loop time marker -- milliseconds 
//int currMyTime = 0; //current time marker -- milliseconds 
//int prev_pos = 0; // stores the position of the motor in the previous loop -- position in degrees
//int target = 0; // target position -- position in degrees
//double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
double current_posLeft = 0; // current position of the motor -- position in degrees
double current_posRight = 0; // current position of the motor -- position in degrees
double countLeft = 0;
double countRight = 0;

double target = 180;
double MOVE_FEET = 5;

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
double right_angular_velocity = 0;
double left_angular_velocity = 0;
double radius = 7.4;
double b = 34.5;
const double Sampling_Interval= 10;


double vel_error;
double wait;


// Proportional controller constants
const double Ki = .33;
const double Kp = 3;

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
double ki_move_forward = .8;
double kp_move_forward = 1;
double speed1;
double speed2;
double speed_error_left = 0;
double speed_error_right = 0;
double angle_multiplier = 0;

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
  //mdRight.init(); //initialize right motor
  delay(500); //delay to let motor initialize

  // i2c interrupt initialization 
  //Wire.begin(PI_ADDRESS);
  //Wire.onReceive(PI_READ);
  //Wire.onRequest(PI_WRITE);
  
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
    right_angular_velocity = 0;
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
  Serial.print(right_angular_velocity);
  Serial.print('\t');
  Serial.print(left_angular_velocity);
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
  // Normalize values over 360 degrees into values from 0-360
  if ( countLeft >= 1600 ) {

    //countLeft = 0;
  }

  A_CURRENT_STATE_LEFT = digitalRead(LEFT_PIN_A);//read state of signal A
  B_CURRENT_STATE_LEFT = digitalRead(LEFT_PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE_LEFT==B_CURRENT_STATE_LEFT){ //if the signals are the same

    countLeft-=1;//subract from the count
  }
  else {//if signals are different
    countLeft+=1;//add 2 to the count
  }
   /*Serial.print( A_CURRENT_STATE );
   Serial.print( "  " );
   Serial.print( B_CURRENT_STATE );
   Serial.print(degree);
   Serial.print( '\n' );// */
}

void RightTick() {

  // Normalize values over 360 degrees into values from 0-360
  if ( countRight >= 1600 ) {

    //countRight = 0;
  }

  A_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_A); //read state of signal A
  B_CURRENT_STATE_RIGHT = digitalRead(RIGHT_PIN_B); //read state of signal B
  
  if (A_CURRENT_STATE_RIGHT==B_CURRENT_STATE_RIGHT){ //if the signals are the same

    countRight+=1;//subract from the count
  }
  else {//if signals are different
    countRight-=1;//add 2 to the count
  }
   /*Serial.print( A_CURRENT_STATE );
   Serial.print( "  " );
   Serial.print( B_CURRENT_STATE );
   Serial.print(degree);
   Serial.print( '\n' );// */
}

void CALCULATIONS(){
  // if statement is responsible for adhering to a sampling interval. The
  //>= Sampling_Interval means sampling intervals of Sampling_Interval
  // milliseconds
  
  if ( (millis() - time_initial) >= Sampling_Interval) {

    time_initial = millis();
    
    // Angular velocity equations
//    right_angular_velocity = ((current_posRight - right_wheel_degrees_initial) / Sampling_Interval) * 3.14159 / 180;
    right_angular_velocity = ((current_posRight - right_wheel_degrees_initial) / (Sampling_Interval / 1000)) * 3.14159 / 180;
    left_angular_velocity = ((current_posLeft - left_wheel_degrees_initial) / (Sampling_Interval / 1000)) * 3.14159 / 180;


//    right_angular_velocity = (18 * ( current_posRight - right_wheel_degrees_initial) ) * (3.14159) / 180;
//    left_angular_velocity = (18 *( current_posLeft - left_wheel_degrees_initial )) * (3.14159) / 180;
    
    right_wheel_degrees_initial = current_posRight;
    left_wheel_degrees_initial = current_posLeft;
    
    // Tracking position of car
    x_pos += ((right_angular_velocity + left_angular_velocity) * radius) /304.8 / Sampling_Interval / 2;
    y_pos += (Sampling_Interval / 1000) * sin( theta ) * (right_angular_velocity + left_angular_velocity) / 2;
    theta += (Sampling_Interval / 1000) * (radius / b) * ( left_angular_velocity - right_angular_velocity) * 180 / 3.14159;

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

//  Serial.print(double(target- theta_l));
//  Serial.print('\n');
  
}


//double m1_velocity = 0;
//double m2_velocity = 0;
//double m1_velocity_error = 0;
//double m2_velocity_error = 0;
//double time_initial_rotate = 0;
//double iterate = 0;
//double ki_move_forward = .8;
//double kp_move_forward = 1;
//double speed1;
//double speed2;
//double speed_error_left = 0;
//double speed_error_right = 0;
//double angle_multiplier = 0;

void MOTOR_MOVE() {

  if ( millis() - time_initial >= Sampling_Interval ) {
    iterate += 1;
    
    time_initial_rotate = millis();
    
    vel_error = (MOVE_FEET - x_pos ) * 3 ;  

    if ( vel_error >= 5 ) {

      vel_error = 5;
      
    }
    speed_error_right = vel_error - right_angular_velocity;
    speed_error_left = vel_error - left_angular_velocity;

    if ( right_angular_velocity > left_angular_velocity ) {
      speed_error_left += abs(speed_error_right - speed_error_left) * 3;
    }
    else {
      speed_error_right += abs(speed_error_left - speed_error_right) * 3;
    }

    if ( abs(speed_error_right) <= .4 ) { 
      speed_error_right = 0;
    }

    if ( abs(speed_error_left) <= .4 ) { 
      speed_error_left = 0;
    }
    
    m1_velocity += ( speed_error_right ) * 10 * iterate / 1000;
    m2_velocity += ( speed_error_left )  * 10 * iterate / 1000;

//    Serial.print('\n');
//    Serial.print(right_angular_velocity);
//    Serial.print('\t');
//    Serial.print(left_angular_velocity);
//    Serial.print('\t');
//    Serial.print(speed_error_right - speed_error_left);
//    Serial.print('\t');
//    Serial.print(x_pos);
//    Serial.print('\t');
//    Serial.print(theta);


//    m1_velocity_error = ( speed_error_right );
//    m2_velocity_error = ( speed_error_left );
  
//    if ( m1_velocity_error >= 250 ) {
//      m1_velocity_error = 250;
//    }
//    
//    if ( m2_velocity_error >= 250 ) {
//      m2_velocity_error = 250;
//    }
  
//  if ( vel >= 225 ) {
//    vel = 225;
//  }

//    speed1 = m1_velocity_error * kp_move_forward + m1_velocity * ki_move_forward;
//    speed2 = m2_velocity_error * kp_move_forward + m2_velocity * ki_move_forward;
//    speed1 = m1_velocity * ki_move_forward;
//    speed2 = m2_velocity * ki_move_forward;
  }

  //speed1 = m1_velocity * ki_move_forward;
  
  speed1 = (vel_error * kp_move_forward + m1_velocity * ki_move_forward);
  speed2 = (vel_error * kp_move_forward + m2_velocity * ki_move_forward);

//  speed1 = speed1 - speed1 * exp(-iterate / 160);
//  speed2 = speed2 - speed2 * exp(-iterate / 160);

  
//  Serial.print('\n');
//  Serial.print( x_pos );
//  angle_multiplier = 100 - theta / 100;
  angle_multiplier = 1;

  md.setM1Speed( speed1 );
  md.setM2Speed( speed2 * angle_multiplier );

}
