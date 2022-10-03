#include <DualMC33926MotorShield.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x02
#define PI_ADDRESS 0x2a


DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
const int PIN_A = 2;
const int PIN_B = 5;
const int PIN_PI = 3;

int number = 0;
byte ardArr[32];
int offset = 2;

// This program uses a state machine to track the encoder’s position
bool A_LAST_STATE = 0;
bool A_CURRENT_STATE = 0;
bool B_CURRENT_STATE = 0;



int count; //motor encoder counts
int intError = 0; //integral error for I in PI controller
int prevMyTime = 0; //previos loop time marker
int currMyTime = 0; //current time marker
int prev_pos = 0; // stores the position of the motor in the previous loop
int target = 0;
double motorVal = 0;//varible to hold the value we write to the motor
double current_pos = 0;

const double Ki = 0.01;//.8
const double Kp = 3.5;

//C code for Arduino
//#get the value that the pi sent to determine which quadrent(only 4 cases) it is in
//void receiveData(int byteCount) {
//  case = wire.read(); #variable case is what quadrent or which of the 4 cases the aruco marker is at
//}

void setup() {
  
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(12,INPUT);
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  
  // Setting up the interrupt handler 
  // Activates whenever the CLK pin on the encoder changes voltage
  attachInterrupt(digitalPinToInterrupt(2), TICK, CHANGE); 
  md.init(); //intialize motor
  delay(500); //delay to let motor initialize

  Wire.begin(PI_ADDRESS);
  Wire.onReceive(PI_READ);
  Wire.onRequest(PI_WRITE);
  //Wire.beginTransmission(PI_ADDRESS);
}

void loop() {
// Nothing happens in the main loop due to interrupt handler
  //Serial.println("Austin is a dumbass");
  
  //current_pos = 0; // This is the current position

  current_pos = double(count)/4.44;//2.22
  //Serial.print('\n');
 //Serial.print(int(current_pos));
  if (current_pos-target <=  10 && current_pos-target >=  -10){// THIS IF STATEMENT IS SUPER DUPER IMPORTANT!!!!!!!!!!!!
    current_pos = target;                // There is some noise in the system that will cause everything to become unstable
    intError = 0;                         // This if statement rejects such noise and keeps the system from running away
  }
  currMyTime = millis();//get current time
  if (currMyTime >= prevMyTime+100){ // every 10th of a second
    intError += (current_pos-target);//*(currMyTime-prevMyTime)*; 
    /*Serial.print(int(current_pos-target)); 
    Serial.print("  ");
    Serial.print( int(current_pos) );
    Serial.print( "  ");/////////////test print statements
    Serial.print(intError);
    Serial.print( '\n' );//*/
    prev_pos = current_pos;//reset prev_pos
    prevMyTime = currMyTime;//reset prevMyTime
    motorVal = double(( target-current_pos )) * Kp + (Ki * double(intError)); //calculate the motor val
    md.setM1Speed( motorVal );// set the motor value (-1024, 1024)
    //Serial.print(motorVal); // print motor value
    //Serial.print('\n');//print new line
  }



}



// Interrupt handler encoder
void PI_READ(int byteCount) { 
  int data = 0;
  data = Wire.read(); // receive a byte as character
  
  /*while (Wire.available()) { // peripheral may send less than requested
      data = Wire.read(); // receive a byte as character
      Serial.print(int(target));  
  }*/
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
  //Serial.print(int(target));  
}

char data[] = "hello";
int index = 0;

//void PI_WRITE() {
//  
//    char data[] = "hello";
//    Wire.write(data[index]);
//    ++index;
//    
//    if (index >= 5) {
//         index = 0;
//    }
//    
//}

void PI_WRITE() {
    static char sendIt[3] = "123";
    static int myNum = 0;
    myNum = int(current_pos);
    switch (myNum/100){
      case 0:
            sendIt[0] = '0';
            break;
       case 1:
            sendIt[0] = '1';   
            myNum = myNum -100;        
           break;
       case 2:
            sendIt[0] = '2';
            myNum = myNum -200;
            break;
       case 3:
            sendIt[0] = '3';
            myNum = myNum -300;
            break;
    }
    switch (myNum/10){
      case 0:
            sendIt[1] = '0';
            break;
       case 1:
            sendIt[1] = '1'; 
            myNum = myNum -10;          
           break;
       case 2:
            sendIt[1] = '2';
            myNum = myNum -20;  
            break;
       case 3:
            sendIt[1] = '3';
            myNum = myNum -30;  
            break;
       case 4:
            sendIt[1] = '4';
            myNum = myNum -40;  
            break;
       case 5:
            sendIt[1] = '5';
            myNum = myNum -50;  
            break;
       case 6:
            sendIt[1] = '6';
            myNum = myNum -60;  
            break;
       case 7:
            sendIt[1] = '7';
            myNum = myNum -70;  
            break;
       case 8:
            sendIt[1] = '8';
            myNum = myNum -80;  
            break;
       case 9:
            sendIt[1] = '9';
            myNum = myNum -90;  
            break;
    }
    switch (myNum){
      case 0:
            sendIt[2] = '0';
            break;
       case 1:
            sendIt[2] = '1'; 
            myNum = myNum -1;          
           break;
       case 2:
            sendIt[2] = '2';
            myNum = myNum -2;  
            break;
       case 3:
            sendIt[3] = '3';
            myNum = myNum -3;  
            break;
       case 4:
            sendIt[4] = '4';
            myNum = myNum -4;  
            break;
       case 5:
            sendIt[5] = '5';
            myNum = myNum -5;  
            break;
       case 6:
            sendIt[6] = '6';
            myNum = myNum -6;  
            break;
       case 7:
            sendIt[7] = '7';
            myNum = myNum -7;  
            break;
       case 8:
            sendIt[8] = '8';
            myNum = myNum -8;  
            break;
       case 9:
            sendIt[9] = '9';
            myNum = myNum -9;  
            break;
    }
    for (int  i = 0; i <= sizeof(sendIt); i++ ) {
      Serial.println(sendIt[i]);
    }
    Wire.write(int(current_pos));
    
}

void TICK() {

  A_CURRENT_STATE = digitalRead(PIN_A);//read state of signal A
  B_CURRENT_STATE = digitalRead(PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE==B_CURRENT_STATE){ //if the signals are the same

    count-=1;//subract from the count
  }
  else {//if signals are different
    count+=1;//add 2 to the count
  }
   /*Serial.print( A_CURRENT_STATE );
   Serial.print( "  " );
   Serial.print( B_CURRENT_STATE );
   Serial.print(degree);
   Serial.print( '\n' );// */
}
