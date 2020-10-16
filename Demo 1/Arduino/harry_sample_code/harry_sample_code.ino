/* a closed loop proportional control
   ms 20200926
*/
// Include Libraries
#include <PinChangeInt.h>

// Driver definitions

// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
#define SHIELD true

//SHIELD Pin varables - cannot be changed
#define motorRIGHTpwm 3
#define motorRIGHTdir 12
#define motorLEFTpwm 11
#define motorLEFTdir 13

//Driver Pin variable - any 4 analog pins (marked with ~ on your board)
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6

// Lab Specific definitions

// Defining these allows us to use letters in place of binary when
// controlling our motors
#define RIGHT 0 // right motor
#define LEFT 1 // left motor
#define pushButton 2 // install a Pullup button with its output into Pin 2
/* If you'd like to use additional buttons as bump sensors, define their pins
    as descriptive names, such as bumperLeft etc.
*/

/*
   Gear ratio:      5:2
   Wheen Diameter:  4.96 cm
   Circumference = 15.58
   Distance per motor rotation = 15.58 * 2 / 5 = 6,23

   Distnce between wheel: 8.5 cm
   Turn Circumference: 26.7
   Quarter turn: 6.675 cm

   One wheel roation = 15.58 cm = 24 encoder counts
   Percent of circle: 15.58 / 26.7 = .5835
   Degrees: .5835 * 360 = 210.06
   to motor: 210.06 * 2 / 4 = 84

*/
//float wheelDiameter = 4.96;
//float wheelCirc = wheelDiameter*M_PI
//float distanceBetweenWheels = 8.5;
//float turnCirc = distanceBetweenWheels * M_PI;
//const float DegreesPerRev = 360 * (wheelCirc / turnCirc);

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 24.0 //was 12 before, but for each hole on the wheel there are 2 counts
#define DistancePerRev      8.1 //cm forwards fo every rotation of the motor. started at 6.23
#define DegreesPerRev       95 //degrees for every rotation of the motor

//These are to build your moves array, a la Lab 2
#define F             0
#define L             1
#define R             -1

#define rightFudge 1.1


// these next two are the digital pins we'll use for the encoders
// You may change these as you see fit.
#define EncoderMotorLeft  7
#define EncoderMotorRight 8

// Proportional Control constants
#define GAIN_RIGHT 7 // right motor
#define GAIN_LEFT 7 // left motor
// how many encoder counts from your goal are accepteable?
#define distTolerance 1

// minimum power settings
// Equal to the min PWM for your robot's wheels to move
// May be different per motor
#define deadband_RIGHT 67 // right motor
#define deadband_LEFT 65 // left motor


// Lab specific variables
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;
int moves[] = {F, L, F, L, F, R, F, F, F, R, F, F, R, F}; // Movements to move through the maze
int fDist = 26;
int rTurn = 90;
int lTurn = 90;


void setup() {
  // set stuff up
  Serial.begin(9600);
  motor_setup();
  pinMode(pushButton, INPUT_PULLUP);

  // add additional pinMode statements for any bump sensors


  // Attaching Wheel Encoder Interrupts
  Serial.println("Encoder Testing Program ");
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.print(EncoderMotorLeft);
  Serial.println();
  pinMode(EncoderMotorLeft, INPUT_PULLUP); //set the pin to input
  // this next line setup the PinChange Interrupt
  PCintPort::attachInterrupt(EncoderMotorLeft, indexLeftEncoderCount, CHANGE);
  /////////////////////////////////////////////////
  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.print(EncoderMotorRight);
  Serial.println();
  pinMode(EncoderMotorRight, INPUT_PULLUP);     //set the pin to input
  PCintPort::attachInterrupt(EncoderMotorRight, indexRightEncoderCount, CHANGE);
} /////////////// end of setup ////////////////////////////////////

/////////////////////// loop() ////////////////////////////////////
void loop()
{
  while (digitalRead(pushButton) == 1); // wait for button push
  while (digitalRead(pushButton) == 0); // wait for button release

  // Loop through entire moves list
  for (int i = 0; i < sizeof(moves) / 2; i++) { //Sizeof() returns number of bytes, each int is two bytes
    if (moves[i] == L) turn(L, lTurn);
    else if (moves[i] == R) turn(R, rTurn);
    else drive(fDist);

    run_motor(RIGHT, 0);
    run_motor(LEFT, 0);
    delay(1000);
  }
  Serial.println("Done with the movement");
}
//////////////////////////////// end of loop() /////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
void drive(float dist)
{
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, errorLeft, errorRight;

  // Find the number of encoder counts based on the distance given, and the
  // configuration of your encoders and wheels
  countsDesired = (int)((dist / DistancePerRev) * EncoderCountsPerRev);

  // reset the current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorLeft = distTolerance + 1;
  errorRight =  distTolerance + 1;

  // Begin PID control until move is complete
  while (errorLeft > distTolerance || errorRight > distTolerance)
  {
    // according to the PID formula, what should the current PWMs be?
    cmdLeft = computeCommand(GAIN_LEFT, deadband_LEFT, errorLeft);
    cmdRight = computeCommand(GAIN_RIGHT, deadband_RIGHT, errorRight);

    // Set new PWMs
    run_motor(LEFT, cmdLeft);
    run_motor(RIGHT, cmdRight);

    // Update encoder error
    errorLeft = countsDesired - leftEncoderCount;
    errorRight = countsDesired - rightEncoderCount;

//    Serial.print(errorLeft);
//    Serial.print(" ");
//    Serial.print(cmdLeft);
//    Serial.print("\t");
//    Serial.print(errorRight);
//    Serial.print(" ");
//    Serial.println(cmdRight);

  }

}
////////////////////////////////////////////////////////////////////////////////


// Write a function for turning with PID control, similar to the drive function
void turn(int dir, int deg) {
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, errorLeft, errorRight;

  // Find the number of encoder counts based on the distance given, and the
  // configuration of your encoders and wheels
  double tmp = (float)deg / DegreesPerRev;
  countsDesired = (int)(tmp * EncoderCountsPerRev);

  // reset the current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorLeft = distTolerance + 1;
  errorRight =  distTolerance + 1;

  // Begin PID control until move is complete
  while (errorLeft > distTolerance || errorRight > distTolerance)
  {
    int changeRIGHT = 1;
    int changeLEFT = 1;
    switch (dir) {
      case L:
        changeLEFT = -1;
        break;
      case R:
        changeRIGHT = -rightFudge;
        break;
    }
    
    // according to the PID formula, what should the current PWMs be?
    cmdLeft = changeLEFT * computeCommand(GAIN_LEFT, deadband_LEFT, errorLeft);
    cmdRight = changeRIGHT * computeCommand(GAIN_RIGHT, deadband_RIGHT, errorRight);

    // Set new PWMs
    run_motor(LEFT, cmdLeft);
    run_motor(RIGHT, cmdRight);

    // Update encoder error
    errorLeft = countsDesired - leftEncoderCount;
    errorRight = countsDesired - rightEncoderCount;


  }
}

////////////////////////////////////////////////////////////////////////////////

int computeCommand(int gain, int deadband, int error)
//  gain, deadband, and error, both are integer values
{
  if (error <= distTolerance) { // if error is acceptable, PWM = 0
    return (0);
  }

  int cmdDir = (gain * error); // Proportional control
  cmdDir = constrain(cmdDir, deadband, 255); // Bind value between motor's min and max
  return cmdDir;
}


//////////////////////////////////////////////////////////

// These are the encoder interupt funcitons, they should NOT be edited

void indexLeftEncoderCount()
{
  leftEncoderCount++;
}
//////////////////////////////////////////////////////////
void indexRightEncoderCount()
{
  rightEncoderCount++;
}
