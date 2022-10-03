#include <Wire.h>
#define SLAVE_ADDRESS 0x2A

int sendData = 4;

void setup() {
    // initialize i2c as slave
    Serial.begin(115200);
    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(PI_WRITE);
    Wire.onReceive(PI_READ); 
}

void loop() {
}

char data = '3';
int index = 0;

// callback for sending data
void PI_WRITE() { 
    Wire.write(data);
    Serial.print("Write ");
    Serial.print(data);
    Serial.print('\n');
 }
 void PI_READ(){
    sendData = Wire.read();
    Serial.print("Read ");
    Serial.print(int(sendData));
    Serial.print('\n');
 }
