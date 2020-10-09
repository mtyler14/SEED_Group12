/*
This code controls motor and spinning wheel. It takes inputs from an I2C bus and an encoder.
The programs outputs a PWM signal to a motor controller and sends the location of the 
wheel over I2C to a raspberry pi.

To use this code, connect the GND, SCL, and SDA lines to the raspberry pi. Plug in the motor
drive to the ardunino and connect the encoder to the pins specified in the code below. 
*/

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x04

// Encoder settings
const int CPR = 3200;
const int enc1A = 2;
const int enc1B = 5;

long enc1Pos = -999;
int encValStart = 0;
int encValEnd = 0;
int encVel = 0;

const double pi = 3.14159;
Encoder enc1(enc1A, enc1B);
// Motor settings
int motorDirection = 7;
int motorVoltage = 9;
int motorEnable = 4;

// Timing variables
int delayVal = 10;
int voltageValue = 128;
long startTime = 0;
long endTime = 0;
long currentTime = 0;
long timeSinceCall = 0;
void reportData();

// Angular measurement variables
double angPos = 0;
double angPosEnd = 0;
char angPosString[5];
char angPosStringPos[5];
double angVel = 0;
double angChange = 0;
double newVelocity = 0;

// PID controller gains
double Kp = 200; // 3082.721; // V/theta
double Ki = 0; // V*s/theta
double Kd = 8; // V/thetas*s

// PID controller variables
double e = 0;
double u = 0;
double derivate = 0;
double I = 0;
double e_past = 0;
int tagNumber = 1;
double r = 4.71;
double y = 0;
double loopSpeed = 50; // 50ms for 20Hz to prevent noise
int delayValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(motorDirection, OUTPUT); //define pin 7 as an output
  pinMode(motorVoltage, OUTPUT); //define pin 9 as an output
  pinMode(motorEnable, OUTPUT); //define pin 4 as an output
  digitalWrite(motorEnable, HIGH);
  pinMode(motorDirection, LOW);
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

}

void loop() {


  currentTime = millis(); //collect time of program in milliseconds
  reportData();
  encValEnd = encValStart;

  e = r - angPos; // error
  derivate = (e - e_past) / (loopSpeed/1000); // approximate the derivative
  e_past = e;
  I = I + (loopSpeed/1000) * e; // summation of error to approximate integral
  if (I > 5) I = 1;
  if (I < -5) I = -1;
  u = e*Kp + Ki * I + Kd * derivate; // PID controller
  //Serial.print(u);
  //Serial.print('\t');
  // Change the directino of the motor based on motor correction
  if (u < 0) {
    digitalWrite(motorDirection, HIGH);
  } else {
    digitalWrite(motorDirection, LOW);
  }
      Serial.print(I);
    Serial.print("\t");
    // Get the absolute values of the correction
  u = abs(u);
  // constrain the correction to valid PWM values
  u = constrain(u, 0, 255);
  analogWrite(motorVoltage, u); // set motor voltage to u

  // Delay a certain amount to keep a constant loop speed
  delayValue = loopSpeed - ((millis() - currentTime) % 50);
  delay(delayValue); // delay accordingly for 50ms

}

void reportData() {
  encValStart = enc1.read(); //read encoder
  //  Serial.print("Time: ");
  //  Serial.print(millis());
  //  Serial.print("\t");
    Serial.print("Encoder Pos: ");
   Serial.println(encValStart);

  encValStart %= CPR;
  encVel = encValStart - encValEnd;

  //calculate angular position
  angPos = (double)(encValStart * 2 * pi) / (CPR);
  //Serial.print("\tAngular Pos: ");
  //  Serial.print(angPos);
  //  Serial.print("\t");


  angChange = angPos - angPosEnd;
  angPosEnd = angPos;

  //Serial.print("\tAngular Pos: ");
  //  Serial.println(angChange);
  //calculate angular velocity
  angVel = (double)(encVel * 2 * pi) / ((loopSpeed) * CPR);
  angVel *= 1000; //In terms of seconds


  //Serial.print("\tAngular Vel: ");
  //Serial.print("     ");
  //Serial.print(angVel);

  // Serial.print("\tSet point: ");
  //Serial.print(r);


}

// callback for received data. Receive data from the raspberry pi
void receiveData(int byteCount) {
  while (Wire.available()) {
    tagNumber = Wire.read();
//    Serial.print("\nI received data. ");
//    Serial.print(tagNumber);
// Set the desired setpoint based on the received tag/quadrant
    if (tagNumber == 1) {
      r = 0;
    } else if (tagNumber == 2) {
      r = M_PI / 2;
    } else if (tagNumber == 3) {
      r = M_PI;
    } else if (tagNumber == 4) {
      r = 3 * M_PI / 2;
    }

  }
}

// callback for sending data
void sendData() {
    // Write data over the I2c bus
    dtostrf(angPos,2,2,angPosString);
    for(int i = 0; i < 5; i++) {
      if(i == 0 && angPosString[0] != '-') {
        angPosStringPos[0] = '+';
        for(int j = 1; j < 5; j++) {
          angPosStringPos[j] = angPosString[i+j-1];
        }
      }
      if (angPosString[0] == '-') Wire.write(angPosString[i]);
      else Wire.write(angPosStringPos[i]);
    }
}
