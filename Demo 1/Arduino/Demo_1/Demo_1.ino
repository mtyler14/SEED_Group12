/*
  This code controls motor and spinning wheel. It takes inputs from an I2C bus and an encoder.
  The programs outputs a PWM signal to a motor controller and sends the location of the
  wheel over I2C to a raspberry pi.
  To use this code, connect the GND, SCL, and SDA lines to the raspberry pi. Plug in the motor
  drive to the ardunino and connect the encoder to the pins specified in the code below.
*/

enum driveState{forward,rotate,beacon,rotateAndForward,idle,done};

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

// Encoder settings
const int CPR = 3200;
const int enc1A = 2;
const int enc1B = 5;
const int enc2A = 3;
const int enc2B = 6;
const int enc2Volt = 11;

String dir = " ";
int count = 0;
int count2 = 0;
int countBeforeDelay = 0;
int countBeforeDelay2 = 0;

int enc1ValA = 0;
int enc1ValB = 0;
int enc2ValA = 0;
int enc2ValB = 0;

double distanceBetweenWheels = 8.5; //inches
double radius = 2.75; //in inches
const double pi = 3.14159;
double circ = radius * pi * 2; //1.44 ft

double voltageControlOne = 0;
double voltageControlTwo = 0;
double changeInVoltage = 0;


// Motor settings
int motorDirection1 = 7;
int motorVoltage1 = 9;
int motorDirection2 = 8;
int motorVoltage2 = 10;
int motorEnable = 4;
int motor1pwm = 0;
int motor2pwm = 0;

// Timing variables
int delayVal = 10;
int voltageValue = 128;
long startTime = 0;
long endTime = -1;
long currentTime = 0;
long timeSinceCall = 0;

// Angular measurement variables
double angPos = 0;
double angPosEnd = 0;
char angPosString[5];
char angPosStringPos[5];
double angVel = 0;
double angChange = 0;
double newVelocity = 0;

// PID controller gains
double Kp = 50; // in PWM, tuned to work from simulation with rise time of 0.179 sec
double Ki = 10; // 
double Kd = 0; // 

// PID controller gains
double KpTwo = 415; // in PWM, tuned to work from simulation with slower rise time of 0.616 sec
double KiTwo = 0; // V*s/theta
double KdTwo = 0; // V/thetas*s

// PID controller one
double e = 0;
double u = 0;
double derivate = 0;
double I = 0;
double e_past = 0;
double rhoDotExperimental = 0;

double loopSpeed = 50; // 50ms for 20Hz to prevent noise
int delayValue = 0;

//PID controller  two
double eTwo = 0;
double uTwo = 0;
double derivateTwo = 0;
double ITwo = 0;
double e_pastTwo = 0;
double phiDotExperimental = 0;
int tagNumber = 1;

void enc1ISR();
int isr1Start = 0;
int isr1End = 0;
int timeSinceInt1 = 0;
double angularVelocity1 = 0;
void enc2ISR();
int isr2Start = 0;
int isr2End = 0;
int timeSinceInt2 = 0;
double angularVelocity2 = 0;

double maxVoltage = 7.4;

double controlVoltage1 = 0;
double controlVoltage2 = 0;
double controlPWMOne = 0;
double controlPWMTwo = 0;

double rhoDot = 0.16; // feet/s for 1V, used for experiments
double phiDot = 0.7; // feet/s for 1V, used for experiments

int countsToDrive = 0;
int countsToDrive2 = 0;
double currPhi = 0;
double desPhi = 0;
driveState state;
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);
  pinMode(enc2Volt, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enc1A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), enc2ISR, CHANGE);

  pinMode(motorDirection1, OUTPUT); //define pin 7 as an output
  pinMode(motorVoltage1, OUTPUT); //define pin 9 as an output
  pinMode(motorDirection2, OUTPUT); //define pin 7 as an output
  pinMode(motorVoltage2, OUTPUT); //define pin 9 as an output

  digitalWrite(enc2Volt, HIGH);
  digitalWrite(motorEnable, HIGH);
  digitalWrite(motorDirection1, LOW);
  digitalWrite(motorDirection2, HIGH); // Should be opposite 1 for same direction motion
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  countsToDrive = moveFeet(5);
  countsToDrive2 = moveFeet(1);
  desPhi = rotateBot(30);
  state = idle;
  
  // define callbacks for i2c communication

  //  Wire.onReceive(receiveData);
  //  Wire.onRequest(sendData);
 // Serial.println();

}

int firstLoop = 0;
void loop() {
//  while(firstLoop < 5){
//    delay(1000);
//    firstLoop++;
//    state = idle;
//  }

 
  switch(state){
    case idle:
      analogWrite(motorVoltage1, 0);
      analogWrite(motorVoltage2, 0);
      delay(1000);
      state = forward;
      Serial.println("Out of Idle");
      break;
    case beacon:
      //need harry help
      //recieveData(byteCount);
      state = forward;
      Serial.println("Out of Beacon");
      break;
    case forward:
     // Serial.println("in fw");
      digitalWrite(motorDirection1, LOW);
      digitalWrite(motorDirection2, HIGH);
      analogWrite(motorVoltage1, controlVoltage1);
      analogWrite(motorVoltage2, controlVoltage2);
      if(count > countsToDrive && count2 > countsToDrive){
         state = rotate;
         count = 0;
         count2 = 0;
         //Serial.println("Out of Forward");
      }
      break;
    case rotate:
     // delay(1000);
      digitalWrite(motorDirection1, LOW);
      digitalWrite(motorDirection2, LOW);      
      analogWrite(motorVoltage1, controlVoltage1);
      analogWrite(motorVoltage2, controlVoltage2);
      currPhi = ((radius/12)/(distanceBetweenWheels/12)) * (((double)(count - count2)/CPR)*2*pi);
      Serial.println(currPhi);
//      count = 0;
//      count2 = 0;
      if( currPhi >= desPhi ){
         delay(2000);
         state = rotateAndForward;
         count = 0;
         count2 = 0;
         Serial.println("Rotate");
      }
      break;

    case rotateAndForward:
      Serial.println("Rotate and Forward");
      digitalWrite(motorDirection1, LOW);
      digitalWrite(motorDirection2, HIGH);
      analogWrite(motorVoltage1, controlVoltage1);
      analogWrite(motorVoltage2, controlVoltage2);
      if(count > countsToDrive2 && count2 > countsToDrive2){
         state = done;
         count = 0;
         count2 = 0;
      }
      break;

    case done: 
    analogWrite(motorVoltage1, 0);
    analogWrite(motorVoltage2, 0);
    break:

    
    default:
      state = idle;
      break;
    
  }
  currentTime = millis(); //collect time of program in milliseconds
  double secTime = (double)currentTime / 1000;
  angularVelocity1 = (count - countBeforeDelay) * 2 * pi / ((double)(currentTime - endTime) * CPR / 1000);
  angularVelocity2 = (count2 - countBeforeDelay2) * 2 * pi / ((double)(currentTime - endTime) * CPR / 1000);

  rhoDotExperimental = (radius / 12) * ((angularVelocity1 + angularVelocity2) / 2); // in feet per second
  phiDotExperimental = (radius / 12) * ((angularVelocity1 - angularVelocity2) / (distanceBetweenWheels / 12)); // in feet per second
  
  // Controller 1 forward velocity
  e = rhoDot - rhoDotExperimental; // error
  
  double temp = loopSpeed / 1000.0;
  derivate = (e - e_past) / temp ; // approximate the derivative
  e_past = e;
  I = I + e * (loopSpeed / 1000); // summation of error to approximate integral
  if (I > 5) I = 1; // prevent integral term wind up
  if (I < -5) I = -1;

  double p_correction = e * Kp;
  double i_correction = I * Ki;
  double d_correction = Kd * derivate;

  u = p_correction + i_correction + d_correction; // PID controller
  // Get the absolute values of the correction
  u = abs(u);
  // constrain the correction to valid PWM values
  u = constrain(u, 0, 255);
 
 
  // Controller 2 rotational velocity
  eTwo = phiDot - phiDotExperimental; // error
  derivateTwo = (eTwo - e_pastTwo) / (loopSpeed/1000.0); // approximate the derivative
  e_pastTwo = eTwo;
  ITwo = ITwo + (loopSpeed/1000) * eTwo; // summation of error to approximate integral
  if (ITwo > 5) ITwo = 1; // prevent integral term wind up
  if (ITwo < -5) ITwo = -1;

  double p_correction2 = eTwo * KpTwo;
  double i_correction2 = ITwo * KiTwo;
  double d_correction2 = KdTwo * derivateTwo;

  uTwo = p_correction2 + i_correction2 + d_correction2; // PID controller
  // Get the absolute values of the correction
  uTwo = abs(uTwo);
  // constrain the correction to valid PWM values
  uTwo = constrain(uTwo, 0, 255);

  controlVoltage1 = (u + uTwo) / 2; // convert controller voltages back to motor voltages
  controlVoltage2 = (u - uTwo) / 2;

  // Delay a certain amount to keep a constant loop speed
  countBeforeDelay = count;
  countBeforeDelay2 = count2;
  endTime = millis();
  delayValue = loopSpeed - ((millis() - currentTime));
  //Serial.println(delayValue);
  delay(delayValue); // delay accordingly for 50ms
  //Serial.print(controlVoltage1); Serial.print(" "); Serial.print(controlVoltage2);Serial.print(" "); Serial.println(state);

}

int moveFeet(double ft) {
  double circFt = circ / 12;
  double rotations = ft / circFt;
  int driveCounts = CPR * rotations;
  return driveCounts;
}


double rotateBot(double degree){
  double phi = (degree * pi)/180;
  return phi;
}

void enc1ISR() {
  enc1ValA = digitalRead(enc1A);
  enc1ValB = digitalRead(enc1B);
  
  if (enc1ValA != enc1ValB) {
    dir = "FW";
    count++;
    count++;
  }
  else {
    dir = "BW";
    count--;
    count--;
  }
}

void enc2ISR() {
  enc2ValA = digitalRead(enc2A);
  enc2ValB = digitalRead(enc2B);
  if (enc2ValA != enc2ValB) {
    dir = "BW";
    count2--;
    count2--;
  }
  else {
    dir = "FW";
    count2++;
    count2++;
  }
}



//void reportData() {
//  encValStart = enc1.read(); //read encoder
//  //  Serial.print("Time: ");
//  //  Serial.print(millis());
//  //  Serial.print("\t");
//    Serial.print("Encoder Pos: ");
//   Serial.println(encValStart);
//
//  encValStart %= CPR;
//  encVel = encValStart - encValEnd;
//
//  //calculate angular position
//  angPos = (double)(encValStart * 2 * pi) / (CPR);
//  //Serial.print("\tAngular Pos: ");
//  //  Serial.print(angPos);
//  //  Serial.print("\t");
//
//
//  angChange = angPos - angPosEnd;
//  angPosEnd = angPos;
//
//  //Serial.print("\tAngular Pos: ");
//  //  Serial.println(angChange);
//  //calculate angular velocity
//  angVel = (double)(encVel * 2 * pi) / ((loopSpeed) * CPR);
//  angVel *= 1000; //In terms of seconds
//
//
//  //Serial.print("\tAngular Vel: ");
//  //Serial.print("     ");
//  //Serial.print(angVel);
//
//  // Serial.print("\tSet point: ");
//  //Serial.print(r);
//
//
//}

//// callback for received data. Receive data from the raspberry pi
//void receiveData(int byteCount) {
//  while (Wire.available()) {
//    tagNumber = Wire.read();
////    Serial.print("\nI received data. ");
////    Serial.print(tagNumber);
//// Set the desired setpoint based on the received tag/quadrant
//    if (tagNumber == 1) {
//      r = 0;
//    } else if (tagNumber == 2) {
//      r = M_PI / 2;
//    } else if (tagNumber == 3) {
//      r = M_PI;
//    } else if (tagNumber == 4) {
//      r = 3 * M_PI / 2;
//    }
//
//  }
//}

// callback for sending data
void sendData() {
  // Write data over the I2c bus
  dtostrf(angPos, 2, 2, angPosString);
  for (int i = 0; i < 5; i++) {
    if (i == 0 && angPosString[0] != '-') {
      angPosStringPos[0] = '+';
      for (int j = 1; j < 5; j++) {
        angPosStringPos[j] = angPosString[i + j - 1];
      }
    }
    if (angPosString[0] == '-') Wire.write(angPosString[i]);
    else Wire.write(angPosStringPos[i]);
  }
}
