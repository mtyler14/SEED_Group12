
![Robot](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/derrick.JPG)
Figure #: Derrick, the robot.

# Demo 2
The objective of Demo 2 is to integrate Derrick's forward and rotational motion into a more complex goal of circling a beacon. Derrick must search and detect the beacon, drive towards it, stop within 1 foot, and drive in a circle around the beacon. He must drive within 2 feet away from the beacon at any time. He must stop within 3 inches of the starting point of the circle. The average time to complete these tasks is of great importance, in addition to reasonable accuracy and success.

## Arduino


## Computer Vision


## Matlab
The control system for Demo 2 utilizes the proportional angular velocity controllers for each motor from Demo 1. The proportional gains used for each controller are slightly reduced to 1.196 V/(rad/s), or 0.08 PWM/(counts/s) as used in Arduino. No integral or derivative gains are added. Figure # shows the unit step response for motor 1, and Figure # shows the unit step response for motor 2. Note that the simulated transfer function is tuned to approximately match the experimental data based on the identification experiments from Demo 1. The unit step responses are assumed to correlate with 0.5 V delivered to each motor.


Instead of adding Ki and Kp gains, the speed of the motors are corrected using Arduino. If the encoder counts (distance) of one motor exceed the other by the set thresholds, then the motor is slowed down.

To further simulate the controllers for Demo 2, each controller's reference response is compared to a faster response (faster speed) to ensure stability. Since the average time is a primary objective, the controllers are compared using a speed about twice as fast as is delivered to both motors. Since Derrick's target speed is approximately 5 feet in 10 seconds to approach the beacon, this speed is translated into a reasonable speed of approximately 1.2 rad/s for each motor. The angular velocities and position are compared for each motor. Figure # shows the controller output with a faster speed for motor 1. Figure # shows the controller output for motor 2. Figure # shows the angular velocities for motor 1. Figure # shows the angular velocities for motor 2. As expected, the results are still reasonably stable and the rise time for each motor is under 0.3 seconds. Figure # shows the angular positions for motor 1. Figure # shows the angular positions for motor 2. Unlike the velocities, the angular position diverges considerably as shown for each motor with a speed of 1.2 rad/s versus 0.4 rad/s for the reference. This is expected, since the controllers do not directly utilize additional gains to correct the speed and position. This response is corrected with the Arduino code for each mode of Derrick's motion. For example, if Derrick circles the beacon and the difference in the motors' speeds (in terms of counts) exceed 10, the controller outputs are scaled by 3. 
