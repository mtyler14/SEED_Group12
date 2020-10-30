![Robot](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%201/images/derrick.JPG)

Figure 1: Robot Configuration

# Demo 1
This folder contains all the code for Demo 1. Each folder contains code related to portions of this project including: computer vision, movement, and simulation.

## Arduino
The arduino code controls predefined motions of the robot. Specifically, linear movements and rotations. Each movement is controlled by proportional velocity controllers.
After a movement has been requested, the number of encoder counts needed to get there is calculated and the wheels turn at a specific speed until each wheel has arrived
near its requested destination. 

## Computer Vision
A Raspberry Pi (RPI) and RPI camera are used to detect ArUco tags. The goal of this detection is to determine the angle between the camera's z axis and the tag.
The camera was calibrated to account for lense distortion at the edge of its field of fiew. This calibration was done by taking multiple images of the checkerboard below
and having OpenCv calculate the intrinisc matrix for the camera.

![checkerboard](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%201/images/checkerboard.JPG)

Figure 2: Checkerboard pattern for calibration.

This ensures that that markers near the edge of the camera's field of view
are reported correctly. The software uses OpenCV's ArUco functions to detect markers and return their location in the image. It also uses the solvePnP function to get the
location and orientation of the tag in the image in the camera's coordinate system. The horizontal and vertical angle of the tag can then be determined with trig.

Once the angle has been detected, it is displayed on and I2C screen connected to the PI. 

## Matlab
The MATLAB files include the Demo 1 main script and two Simulink models for each of the PID controllers. The script file contains the step response data for the angular velocities for the motors, the tuned first order transfer functions, and the implementation of the Simulink models for the controllers. The simulated transfer functions are determined based on the settling time and time constant for each motor. The experimental data for the step responses is compared to the simulated transfer functions to ensure an accurate match and the simulated transfer functions are then used in the close loop block diagrams for the Simulink models. Figures 3 and 4 show the step response comparisons for motor 1 and motor 2. Proportional controllers are used for each motor, where the gain represents the motor voltage required per radian per second. Figures 5 and 6 show the proportional controller step responses. The gains are converted to voltage per counts per second upon implementation with the arduino code.
![Motor 1 Transfer Function](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%201/images/motor1.png)
Figure 3: Motor 1 Transfer Function Comparison
![Motor 2 Transfer Function](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%201/images/motor2.png)
Figure 4: Motor 2 Transfer Function Comparison
![Motor 1 Proportional Controller](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%201/images/motor1P.png)
Figure 5: Motor 1 Proportional Controller
![Motor 2 Proportional Controller](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%201/images/motor2P.png)
Figure 6: Motor 2 Proportional Controller
