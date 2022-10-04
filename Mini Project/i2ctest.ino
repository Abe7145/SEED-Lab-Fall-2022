#include <Wire.h>
#define SLAVE_ADDRESS 0x2A

void setup() {
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(sendData); 
}

void loop() {
}

char data[] = "hello";
int index = 0;

// callback for sending data
void sendData() { 
    Wire.write(data[index]);
    ++index;
    if (index >= 5) {
         index = 0;
    }
 }
