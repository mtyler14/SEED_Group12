/*
  This code controls motor and spinning wheel. It takes inputs from an I2C bus and an encoder.
  The programs outputs a PWM signal to a motor controller and sends the location of the
  wheel over I2C to a raspberry pi.
  To use this code, connect the GND, SCL, and SDA lines to the raspberry pi. Plug in the motor
  drive to the ardunino and connect the encoder to the pins specified in the code below.
*/


// Encoder settings
const int CPR = 3200;
const int enc1A = 2;
const int enc1B = 5;
const int enc2A = 3;
const int enc2B = 6;
const int enc2Volt = 11;

String dir = " ";
int countLeft = 0;
int countRight = 0;
int countLeftBeforeDelay = 0;
int countRightBeforeDelay = 0;

int encLeftValA = 0;
int encLeftValB = 0;
int enc2ValA = 0;
int enc2ValB = 0;

double distanceBetweenWheels = 8.5; //inches
double radius = 3; //in inches
double circ = radius * M_PI * 2; //1.44 ft

// Motor settings
int motorDirectionLeft = 7;
int motorVoltageLeft = 9;
int motorDirectionRight = 8;
int motorVoltageRight = 10;
int motorEnable = 4;

// Timing variables
long endTime = -1;
long currentTime = 0;

// Angular measurement variables
double angPos = 0;
double angPosEnd = 0;
char angPosString[5];
char angPosStringPos[5];
double angVel = 0;
double angChange = 0;
double newVelocity = 0;

// PID controller gains
double KpLeft = .1; // in PWM, tuned to work from simulation with rise time of 0.179 sec
double KiLeft = 0; //
double KdLeft = 0; //

// PID controller gains
double KpRight = .1; // in PWM, tuned to work from simulation with slower rise time of 0.616 sec
double KiRight = 0; // V*s/theta
double KdRight = 0; // V/thetas*s

// PID controller one
double errorLeft = 0;
double correctionLeft = 0;
double derivativeLeft = 0;
double integralLeft = 0;
double oldErrorLeft = 0;
double leftAngularSpeed = 0;

double loopSpeed = 50; // 50ms for 20Hz to prevent noise
int delayValue = 0;

//PID controller  two
double errorRight = 0;
double correctionRight = 0;
double derivativeRight = 0;
double integralRight = 0;
double oldErrorRight = 0;
double rightAngularSpeed = 0;

void encLeftISR();
void encRightISR();

double controlVoltage1 = 0;
double controlVoltage2 = 0;

// Variables to control the speed of each wheel in counts per second
double desiredLeftAngularSpeed = CPR / 2; // feet/s for 1V, used for experiments
double desiredRightAngularSpeed = CPR / 2; // feet/s for 1V, used for experiments

// Variables to control the distance the robot moves
int desiredCountsLeft = 0;
int desiredCountsRight = 0;

int tolerance = 5;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);
  pinMode(enc2Volt, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enc1A), encLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), encRightISR, CHANGE);

  pinMode(motorDirectionLeft, OUTPUT); //define pin 7 as an output
  pinMode(motorVoltageLeft, OUTPUT); //define pin 9 as an output
  pinMode(motorDirectionRight, OUTPUT); //define pin 7 as an output
  pinMode(motorVoltageRight, OUTPUT); //define pin 9 as an output

  digitalWrite(enc2Volt, HIGH);
  digitalWrite(motorEnable, HIGH);
  digitalWrite(motorDirectionLeft, LOW);
  digitalWrite(motorDirectionRight, HIGH); // Should be opposite 1 for same direction motion
  // initialize i2c as slave

  drive(10);
}

void loop() {

}

void drive(int feet) {
  countLeft = 0;
  countRight = 0;
  desiredCountsLeft = feet2Counts(feet);
  desiredCountsRight = feet2Counts(feet);
  while ((desiredCountsLeft - countLeft) > tolerance || (desiredCountsRight - countRight) > tolerance) {
    currentTime = millis(); //collect time of program in milliseconds
    Serial.println(countLeft - countRight);


    // Get the motor angunlar speeds in counts per second
    leftAngularSpeed = (countLeft - countLeftBeforeDelay) / ((double)(currentTime - endTime) / 1000);
    rightAngularSpeed = (countRight - countRightBeforeDelay) / ((double)(currentTime - endTime) / 1000);

    //////////////////////////////////////////////// Left motor control calculations ///////////////////////////////////////////////////////
    // Left motor error in angular speed
    errorLeft = desiredLeftAngularSpeed - leftAngularSpeed; // error

    double temp = loopSpeed / 1000.0;
    derivativeLeft = (errorLeft - oldErrorLeft) / temp ; // approximate the derivative
    oldErrorLeft = errorLeft;
    integralLeft = integralLeft + errorLeft * (loopSpeed / 1000); // summation of error to approximate integral
    if (integralLeft > 5) integralLeft = 5; // prevent integral term wind up
    if (integralLeft < -5) integralLeft = -5;

      double pCorrectionLeft = errorLeft * KpLeft;
      double iCorrectionLeft = integralLeft * KiLeft;
      double dCorrectionLeft = KdLeft * derivativeRight;

      correctionLeft = pCorrectionLeft + iCorrectionLeft + dCorrectionLeft; // PID controller
      // Get the absolute values of the correction
      correctionLeft = abs(correctionLeft);
      // constrain the correction to valid PWM values
      correctionLeft = constrain(correctionLeft, 0, 255);

      //////////////////////////////////////////////// Right motor control calculations ///////////////////////////////////////////////////////
      // Controller 2 rotational velocity
      errorRight = desiredRightAngularSpeed - rightAngularSpeed; // error
      derivativeRight = (errorRight - oldErrorRight) / (loopSpeed / 1000.0); // approximate the derivative
      oldErrorRight = errorRight;
      integralRight = integralRight + (loopSpeed / 1000) * errorRight; // summation of error to approximate integral
      if (integralRight > 5) integralRight = 5; // prevent integral term wind up
        if (integralRight < -5) integralRight = -5;

          double pCorrectionRight = errorRight * KpRight;
          double iCorrectionRight = integralRight * KiRight;
          double dCorrectionRight = KdRight * derivativeRight;

          correctionRight = pCorrectionRight + iCorrectionRight + dCorrectionRight; // PID controller
          // Get the absolute values of the correction
          correctionRight = abs(correctionRight);
          // constrain the correction to valid PWM values
          correctionRight = constrain(correctionRight, 0, 255);

          // Delay a certain amount to keep a constant loop speed
          countLeftBeforeDelay = countLeft;
          countRightBeforeDelay = countRight;
          endTime = millis();
          delayValue = loopSpeed - ((millis() - currentTime));

            if ((countLeft - countRight) > 10) {
              correctionLeft /= 2;
            }
    if ((countRight - countLeft) > 10) {
      correctionRight /= 2;
    }

    analogWrite(motorVoltageLeft, correctionLeft);
    analogWrite(motorVoltageRight, correctionRight);

    //    Serial.print(correctionLeft); Serial.print('\t'); Serial.println(correctionRight);

    delay(delayValue); // delay accordingly for 50ms
  }

  Serial.println("Finished with the first movement");
  analogWrite(motorVoltageLeft, 0);
  analogWrite(motorVoltageRight, 0);
}

int feet2Counts(double ft) {
  double circFt = circ / 12;
  double rotations = ft / circFt;
  int driveCounts = CPR * rotations;
  return driveCounts;
}

double degrees2Radians(double degree) {
  double phi = (degree * M_PI) / 180;
  return phi;
}

void encLeftISR() {
  encLeftValA = digitalRead(enc1A);
  encLeftValB = digitalRead(enc1B);

  if (encLeftValA != encLeftValB) {
    dir = "FW";
    countLeft++;
    countLeft++;
  }
  else {
    dir = "BW";
    countLeft--;
    countLeft--;
  }
}

void encRightISR() {
  enc2ValA = digitalRead(enc2A);
  enc2ValB = digitalRead(enc2B);
  if (enc2ValA != enc2ValB) {
    dir = "BW";
    countRight--;
    countRight--;
  }
  else {
    dir = "FW";
    countRight++;
    countRight++;
  }
}
