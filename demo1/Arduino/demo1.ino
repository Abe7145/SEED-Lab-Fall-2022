#include <DualMC33926MotorShield.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x02
#define PI_ADDRESS 0x20


DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
const int PIN_A_r = 2;
const int PIN_B_r = 5;

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

int intError = 0; //integral error for I in PI controller -- tenths of a degree second       
int prevMyTime = 0; //previous loop time marker -- milliseconds 
int currMyTime = 0; //current time marker -- milliseconds 
int prev_pos = 0; // stores the position of the motor in the previous loop -- position in degrees
int target = 0; // target position -- position in degrees
double motorVal = 0;//variable to hold the value we write to the motor -- units in degrees
double current_pos = 0; // current position of the motor -- position in degrees

// Proportional controller constants
const double Ki = 0.01;
const double Kp = 3.5;

//C code for Arduino
//#get the value that the pi sent to determine which quadrent(only 4 cases) it is in
//void receiveData(int byteCount) {
//  case = wire.read(); #variable case is what quadrent or which of the 4 cases the aruco marker is at
//}

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
  Wire.onRequest(PI_WRITE);
  
}

void loop() {
  // Nothing happens in the main loop due to interrupt handler
  // Serial.println("Austin is a dumbass");

  // Normalizing the unitless encoder counts into a 360 degree rotation
  current_pos = double(count)/4.44;//2.22

  
  if (current_pos-target <=  10 && current_pos-target >=  -10){// THIS IF STATEMENT IS SUPER DUPER IMPORTANT!!!!!!!!!!!!
    current_pos = target;                // There is some noise in the system that will cause everything to become unstable
    intError = 0;                         // This if statement rejects such noise and keeps the system from running away
  }
  
  currMyTime = millis();//get current time
  if (currMyTime >= prevMyTime+100){ // sample every 10th of a second

    // calculating integral error
    intError += (current_pos-target);

    // storing dt
    prev_pos = current_pos;//reset prev_pos
    prevMyTime = currMyTime;//reset prevMyTime

    // use built in shield library to move the motor 
    // takes in values from -400 to 400
    // uses a built in PWM to control the voltage supplied to the motor
    motorVal = double(( target-current_pos )) * Kp + (Ki * double(intError)); //calculate the motor val
    md.setM1Speed( motorVal );
    
  }

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

//// instantiate index to iterate through charArray inside of the PI_WRITE loop
//int index = 0;
//
//// i2c interrupt handler for request request from PI
//void PI_WRITE() {
//
//    // if current position is negative, normalize to a degree value from 0-360
//    if ( current_pos < 0 ) {
//
//      current_pos = 360 + current_pos;
//      
//    }
//
//    // Convert current_pos integer into a character array
//    // I2C wire write is only compatible with bytes
//    // the serial protocol gets confused with integers - must use a char array
//    String myNum = String(current_pos);
//    int string_length = myNum.length() + 1;
//    char charArray[string_length];
//    myNum.toCharArray(charArray, string_length);
//
//    // Send value to Pi
//    Wire.write(charArray[index]);
//    ++index;
//    if (index >= 3) {
//         index = 0;
//    }
//    
//}

// Encoder interrupt
void TICK_RIGHT() {

  // Normalize values over 360 degrees into values from 0-360
  if ( count_r >= 1600 ) {

    count_r = 0;
  }

  A_CURRENT_STATE_r = digitalRead(PIN_A);//read state of signal A
  B_CURRENT_STATE_r = digitalRead(PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE_r==B_CURRENT_STATE_r){ //if the signals are the same

    count_r-=1;//subract from the count
  }
  else {//if signals are different
    count_r+=1;//add 2 to the count
  }

}

void TICK_LEFT() {

  // Normalize values over 360 degrees into values from 0-360
  if ( count_l >= 1600 ) {

    count_l = 0;
  }

  A_CURRENT_STATE_l = digitalRead(PIN_A);//read state of signal A
  B_CURRENT_STATE_l = digitalRead(PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE_l==B_CURRENT_STATE_l){ //if the signals are the same

    count_l-=1;//subract from the count
  }
  else {//if signals are different
    count_l+=1;//add 2 to the count
  }

}
