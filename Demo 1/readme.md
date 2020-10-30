# Demo 1
This folder contains all the code for Demo 1. Each folder contains code related to portions of this project including: computer vision, movement, and simulation.

## Arduino
The arduino code controls predefined motions of the robot. Specifically, linear movements and rotations. Each movement is controlled by proportional velocity controllers.
After a movement has been requested, the number of encoder counts needed to get there is calculated and the wheels turn at a specific speed until each wheel has arrived
near its requested destination. 

## Computer Vision
A Raspberry Pi (RPI) and RPI camera are used to detect ArUco tags. The goal of this detection is to determine the angle between the camera's z axis and the tag.
The camera was calibrated to account for lense distortion at the edge of its field of fiew. This ensures that that markers near the edge of the camera's field of view
are reported correctly. The software uses OpenCV's ArUco functions to detect markers and return their location in the image. It also uses the solvePnP function to get the
location and orientation of the tag in the image in the camera's coordinate system. The horizontal and vertical angle of the tag can then be determined with trig.

Once the angle has been detected, it is displayed on and I2C screen connected to the PI. 

## Matlab
Rachel, if you could add an explanation here.
