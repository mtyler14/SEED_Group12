/*
Exercise 1
Receives an integer value from the Raspberry Pi through I2C, adds 5 to the
integer, and sends in back to the Pi
*/

#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount) {
  while(Wire.available()) {
    number = Wire.read();
    Serial.print("data received: ");
    Serial.println(number);

  }
}

// callback for sending data
void sendData(){
  // add 5 to the number that was sent over by the Pi
  Wire.write(number+4);
}
