/*
  This code controls motor and spinning wheel. It takes inputs from an I2C bus and an encoder.
  The programs outputs a PWM signal to a motor controller and sends the location of the
  wheel over I2C to a raspberry pi.
  To use this code, connect the GND, SCL, and SDA lines to the raspberry pi. Plug in the motor
  drive to the ardunino and connect the encoder to the pins specified in the code below.
*/

#define FORWARDS  1
#define ROTATION  0

// Encoder settings
const int CPR = 3200;
const int encRightA = 2;
const int encRightB = 5;
const int encLeftA = 3;
const int encLeftB = 6;
const int enc2Volt = 11;  //What is this?

String dir = " ";
int countRight = 0;
int countLeft = 0;
int countRightBeforeDelay = 0;
int countLeftBeforeDelay = 0;

// Variables to check the encoder's direction of rotation
int encRightValA = 0;
int encRightValB = 0;
int encLeftValA = 0;
int encLeftValB = 0;

// Robot parameters
double distanceBetweenWheels = 9.18; //inches
double radius = 3; //in inches
double wheelCircumference = radius * M_PI * 2; //1.44 ft

// Motor pins
int motorRightDirection = 7;
int motorRight = 9;
int motorLeftDirection = 8;
int motorLeft = 10;
int motorEnable = 4;

// Timing variables
long endTime = -1;
long currentTime = 0;

// Right velocity PID controller
double KpRight = .1;
double KiRight = 0;
double KdRight = 0;

double errorRight = 0;
double controllerOutputRight = 0;
double derivativeRight = 0;
double integralRight = 0;
double oldErrorRight = 0;
double rightAngularSpeed = 0;

// Right positions controller
double KpRightPos = .1;
double KiRightPos = 0;
double KdRightPos = 0;

double errorRightPos = 0;
double controllerOutputRightPos = 0;
double derivativeRightPos = 0;
double integralRightPos = 0;
double oldErrorRightPos = 0;

/////////////////////////////////////////////////////////
// Setting integral and derivative gains might reduce the need to slow one motor down if it gets ahead of the other
/////////////////////////////////////////////////////////

// Left velocity PID controller
double KpLeft = .1;
double KiLeft = 0;
double KdLeft = 0;

double errorLeft = 0;
double controllerOutputLeft = 0;
double derivativeLeft = 0;
double integralLeft = 0;
double oldErrorLeft = 0;
double LeftAngularSpeed = 0;

// Left positional controller
double KpLeftPos = .1;
double KiLeftPos = 0;
double KdLeftPos = 0;

double errorLeftPos = 0;
double controllerOutputLeftPos = 0;
double derivativeLeftPos = 0;
double integralLeftPos = 0;
double oldErrorLeftPos = 0;

double loopSpeed = 50; // 50ms for 20Hz to prevent noise
int delayValue = 0;

// Interrupt routines for the encoders
void encRightISR();
void encLeftISR();

// Variables to control the speed of each wheel in counts per second
double desiredRightAngularSpeed = CPR; // feet/s for 1V, used for experiments
double desiredLeftAngularSpeed = CPR; // feet/s for 1V, used for experiments

// Variables to control the distance the robot moves
int desiredCountsRight = 0;
int desiredCountsLeft = 0;

// Encoder must read within this values of the desired counts to be considered position reached
int tolerance = 5;

void setup() {
  Serial.begin(9600);
  pinMode(encRightA,  INPUT_PULLUP);
  pinMode(encLeftA,   INPUT_PULLUP);
  pinMode(encRightB,  INPUT_PULLUP);
  pinMode(encLeftB,   INPUT_PULLUP);
  pinMode(enc2Volt,   OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encRightA), encRightISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLeftA),  encLeftISR,   CHANGE);

  pinMode(motorRightDirection,  OUTPUT);
  pinMode(motorRight,           OUTPUT);
  pinMode(motorLeftDirection,   OUTPUT);
  pinMode(motorLeft,            OUTPUT);

  digitalWrite(enc2Volt, HIGH);
  digitalWrite(motorEnable, HIGH);
  // Initialize the motors running forwards
  motorsIdle();

   move(10, FORWARDS);
   move(180, ROTATION);
  
}

void loop() {
}

// Set the motors to move forwards
void motorsForwards() {
  digitalWrite(motorRightDirection, LOW);
  digitalWrite(motorLeftDirection, HIGH);
}


// Set the motors to turn the robot ccw
void motorsCounterclockwise() {
  digitalWrite(motorRightDirection, HIGH);
  digitalWrite(motorLeftDirection, HIGH);
}


// Set the motors to turn CW
void motorsClockwise() {
  digitalWrite(motorRightDirection, LOW);
  digitalWrite(motorLeftDirection, LOW);
}


// Turn the motors off
void motorsIdle() {
  analogWrite(motorLeft, 0);
  analogWrite(motorRight, 0);
  delay(750);
}


void move(int distance, int forwardsOrDegrees) {
  // Initialize the counts to zero before movements
  countRight = 0;
  countLeft = 0;

  // Set counts and directions
  switch (forwardsOrDegrees) {
    case FORWARDS:
      motorsForwards();
      desiredCountsRight = feet2Counts(distance);
      desiredCountsLeft = feet2Counts(distance);
      break;
    case ROTATION:
      desiredCountsRight = degrees2Counts(distance);
      desiredCountsLeft = degrees2Counts(distance);

      if (distance > 0) {
        motorsClockwise();
        // Set the desired counts to be negative for the motor in reverse
        desiredCountsRight *= -1;
      }
      else {
        motorsCounterclockwise();
        // Set the desired counts to be negative for the motor in reverse
        desiredCountsLeft *= -1;
      }
      break;
    default:
      motorsIdle();
      break;
  }

  // While the encoder counts are more than a certain amount from the target
  while (abs((abs(desiredCountsRight) - abs(countRight))) > tolerance || abs((abs(desiredCountsLeft) - abs(countLeft))) > tolerance) {
    currentTime = millis(); //collect time of program in milliseconds

//    If both motors have gone too far then stop then exit the loop
    if (abs(countRight) > abs(desiredCountsRight) || abs(countLeft) > abs(desiredCountsLeft)){
      break;
    }

    // Ensures that the same encoder counts are used for each calculation. Otherwise the encoder counts could change during the loop
    int currentCountsRight = countRight;
    int currentCountsLeft = countLeft;

    //////////////////////////////////////////////// Right motor positional Calculations ///////////////////////////////////////////////////////
    errorRightPos = currentCountsRight - desiredCountsRight;
    derivativeRightPos = (errorRightPos - oldErrorRightPos) / (loopSpeed / 1000.0);
    integralRightPos += errorRightPos * (loopSpeed / 1000.0);
    if (integralRightPos > 5) integralRightPos = 5; // prevent integral term wind up
    if (integralRightPos < -5) integralRightPos = -5;

    controllerOutputRightPos = errorRightPos * KpRightPos + derivativeRightPos * KdRightPos + integralRightPos * KiRightPos;
    controllerOutputRightPos = abs(controllerOutputRightPos);
    controllerOutputRight = constrain(controllerOutputRightPos, 0, CPR / 2);
    // This output should be the desiredRightAngularSpeed
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////// Right motor control calculations ///////////////////////////////////////////////////////
    // I think this should use loopTime. The speed is measured as change in counts per change in time. The change in counts in measured every loopTime ms.
    rightAngularSpeed = abs((currentCountsRight - countRightBeforeDelay) / ((double)(currentTime - endTime) / 1000));
    // Right motor error in angular speed
    errorRight = desiredRightAngularSpeed - rightAngularSpeed; // error

    derivativeRight = (errorRight - oldErrorRight) / (loopSpeed / 1000.0) ; // approximate the derivative
    integralRight = integralRight + errorRight * (loopSpeed / 1000); // summation of error to approximate integral
    if (integralRight > 5) integralRight = 5; // prevent integral term wind up
    if (integralRight < -5) integralRight = -5;

    // Calculate the controller output and constrain it
    controllerOutputRight = errorRight * KpRight + integralRight * KiRight + KdRight * derivativeLeft; // PID controller
    controllerOutputRight = abs(controllerOutputRight);
    controllerOutputRight = constrain(controllerOutputRight, 0, 255);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////// Left motor positional Calculations ///////////////////////////////////////////////////////
    errorLeftPos = currentCountsLeft - desiredCountsLeft;
    derivativeLeftPos = (errorLeftPos - oldErrorLeftPos) / (loopSpeed / 1000.0);
    integralLeftPos += errorLeftPos * (loopSpeed / 1000.0);
    if (integralLeftPos > 5) integralLeftPos = 5; // prevent integral term wind up
    if (integralLeftPos < -5) integralLeftPos = -5;

    controllerOutputLeftPos = errorLeftPos * KpLeftPos + derivativeLeftPos * KdLeftPos + integralLeftPos * KiLeftPos;
    controllerOutputLeftPos = abs(controllerOutputLeftPos);
    controllerOutputLeftPos = constrain(controllerOutputLeftPos, 0, CPR / 2);
    // This output should be desiredLeftAngularSpeed
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////// Left motor control calculations ///////////////////////////////////////////////////////
    // I think this should use loopTime. The speed is measured as change in counts per change in time. The change in counts in measured every loopTime ms.
    LeftAngularSpeed = abs((currentCountsLeft - countLeftBeforeDelay) / ((double)(currentTime - endTime) / 1000));
    // Left motor error in angular speed
    errorLeft = desiredLeftAngularSpeed - LeftAngularSpeed; // error

    derivativeLeft = (errorLeft - oldErrorLeft) / (loopSpeed / 1000.0); // approximate the derivative
    integralLeft = integralLeft + (loopSpeed / 1000) * errorLeft; // summation of error to approximate integral
    if (integralLeft > 5) integralLeft = 5; // prevent integral term wind up
    if (integralLeft < -5) integralLeft = -5;

    // Calculate the controller output and constrain it
    controllerOutputLeft = errorLeft * KpLeft + integralLeft * KiLeft + KdLeft * derivativeLeft; // PID controller
    controllerOutputLeft = abs(controllerOutputLeft);
    controllerOutputLeft = constrain(controllerOutputLeft, 0, 255);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // If one motor has gone farther than the other then slow it down until the motors are back in sync
    if ((abs(currentCountsRight) - abs(currentCountsLeft)) > 50) {
      controllerOutputRight /= 3;
    }
    else if ((abs(currentCountsLeft) - abs(currentCountsRight)) > 50) {
      controllerOutputLeft  /= 3;
    }

    // Send the commands to the motor
    analogWrite(motorRight, controllerOutputRight);
    analogWrite(motorLeft, controllerOutputLeft);

    oldErrorRight = errorRight;
    oldErrorLeft = errorLeft;

    countRightBeforeDelay = currentCountsRight;
    countLeftBeforeDelay = currentCountsLeft;
    // Delay a certain amount to keep a constant loop speed
    endTime = millis();
    delayValue = loopSpeed - ((millis() - currentTime));

    delay(delayValue); // delay accordingly for 50ms
  }
  Serial.println("The movement finished");
  motorsIdle();
}


// Convert degrees to encoder counts
int degrees2Counts(int deg) {
  // Circumference traced by wheels
  double pathLength = distanceBetweenWheels * M_PI;         // cm
  // Ratio of wheel circumference to path length
  double rotationRatio = wheelCircumference / pathLength;   // unitless
  // Degrees per rotation
  double degreePerRotation = 360 * rotationRatio;           // degrees
  // Degrees per encoder count
  double countsPerDegree = CPR / degreePerRotation;         // counts / degree
  // Counts for desired rotation
  double desiredCounts = countsPerDegree * deg;         // counts
  // Return the desired counts
  return desiredCounts;
}


// Convert feet encoder counts
int feet2Counts(double ft) {
  double circFt = wheelCircumference / 12;
  double rotations = ft / circFt;
  int driveCounts = CPR * rotations;
  return driveCounts;
}

double degrees2Radians(double degree) {
  double phi = (degree * M_PI) / 180;
  return phi;
}


void encRightISR() {
  encRightValA = digitalRead(encRightA);
  encRightValB = digitalRead(encRightB);

  if (encRightValA != encRightValB) {
    dir = "FW";
    countRight++;
    countRight++;
  }
  else {
    dir = "BW";
    countRight--;
    countRight--;
  }
}


void encLeftISR() {
  encLeftValA = digitalRead(encLeftA);
  encLeftValB = digitalRead(encLeftB);
  if (encLeftValA != encLeftValB) {
    dir = "BW";
    countLeft--;
    countLeft--;
  }
  else {
    dir = "FW";
    countLeft++;
    countLeft++;
  }
}
