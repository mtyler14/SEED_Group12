
![Robot](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/derrick.JPG)

Figure #: Derrick, the robot.

# Demo 2
The objective of Demo 2 is to integrate Derrick's forward and rotational motion into a more complex goal of circling a beacon. Derrick must search and detect the beacon, drive towards it, stop within 1 foot, and drive in a circle around the beacon. He must drive within 2 feet away from the beacon at any time. He must stop within 3 inches of the starting point of the circle. The average time to complete these tasks is of great importance, in addition to reasonable accuracy and success.

## Arduino


## Computer Vision


## Matlab
The control system for Demo 2 utilizes the proportional angular velocity controllers for each motor from Demo 1. The proportional gains used for each controller are slightly reduced to 1.196 V/(rad/s), or 0.08 PWM/(counts/s) as used in Arduino. No integral or derivative gains are added. Figure # shows the unit step response for motor 1, and Figure # shows the unit step response for motor 2. Note that the simulated transfer function is tuned to approximately match the experimental data based on the identification experiments from Demo 1. The unit step responses are assumed to correlate with 0.5 V delivered to each motor.

![Motor1 tf](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_tf.jpg)

Figure #: The tuned step response for motor 1.

![Motor2 tf](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_tf.jpg)

Figure #: The tuned step response for motor 2.

To ensure an accurate position, the angular positions are also simulated by adding an integrator to the transfer functions for the velocities. Figure # shows the position for motor 1. Figure # shows the position for motor 2.

![Motor1 position](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_pos.jpg)

Figure #: The angular position for motor 1.

![Motor2 position](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_pos.jpg)

Figure #: The angular position for motor 2.

Instead of adding Ki and Kp gains, the speed of the motors are corrected using Arduino. If the encoder counts (distance) of one motor exceed the other by the set thresholds, then the motor is slowed down.

To further simulate the controllers for Demo 2, each controller's reference response is compared to a faster response (faster speed) to ensure stability. Since the average time is a primary objective, the controllers are compared using a speed about twice as fast as is delivered to both motors. Since Derrick's target speed is approximately 5 feet in 10 seconds to approach the beacon, this speed is translated into a reasonable speed of approximately 1.2 rad/s for each motor. The angular velocities and position are compared for each motor. Figure # shows the Simulink block diagram for the faster speed to motor 1. A step response with twice the gain is added 10ms, or 1 sampling time duration, after the reference step response to account for the faster speed. The same procedure is used for motor 2.

![Motor1 block](

Figure # shows the controller output with a faster speed for motor 1. Figure # shows the controller output for motor 2. Figure # shows the angular velocities for motor 1. Figure # shows the angular velocities for motor 2. As expected, the results are still reasonably stable and the rise time for each motor is under 0.3 seconds. 

![Motor1 controller fast](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_control.jpg)

Figure #: The proportional controller for reference versus faster speed for motor 1.

![Motor2 controller fast](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_control.jpg)

Figure #: The proportional controller for reference versus faster speed for motor 2.

![Motor1 velocity](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_veloc.jpg)

Figure #: The angular velocities for reference versus faster speed for motor 1.

![Motor2 velocity](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_veloc.jpg)

Figure #: The angular velocities for reference versus faster speed for motor 2.

Figure # shows the angular positions for motor 1. Figure # shows the angular positions for motor 2. The angular position diverges considerably as shown for each motor with a speed of 1.2 rad/s versus 0.6 rad/s for the reference. This is expected, since the controllers do not directly utilize additional gains to correct the speed and position. This response is corrected with the Arduino code for each mode of Derrick's motion. For example, if Derrick circles the beacon and the difference in the motors' speeds (in terms of counts) exceed 10, the controller outputs are scaled by 3. 

![Motor1 position speed](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_pos_speed.jpg)

Figure #: The angular position for reference versus faster speed for motor 1.

![Motor2 position speed](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_pos_speed.jpg)

Figure #: The angular position for reference versus faster speed for motor 2.

Hence, the control system is based on the encoder counts and angular velocity of each motor and the resulting position. The target objectives are successfully met with the necessary corrections to the speed based on the comparison of the motors' responses.
