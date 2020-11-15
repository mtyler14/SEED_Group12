
![Robot](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/derrick.JPG)

Figure #: Derrick, the robot.

# Demo 2
The objective of Demo 2 is to integrate Derrick's forward and rotational motion into a more complex goal of circling a beacon. Derrick must search and detect the beacon, drive towards it, stop within 1 foot, and drive in a circle around the beacon. He must drive within 2 feet away from the beacon at any time. He must stop within 3 inches of the starting point of the circle. The average time to complete these tasks is of great importance, in addition to reasonable accuracy and success.

## Arduino


## Computer Vision
The computer vision portion of this project is very similar to the computer vision code used in [Demo 1](https://github.com/mtyler14/SEED_Group12/tree/master/Demo%201). 
The code works by using OpenCV to detect ArUco markers. The 3D pose of the marker is determined and the angle and distance from the camera is calculated. The angle is found with
the inverse tangent of the z position divided by the x position. The distance is found by calculating the magnitude of the z and x positions. Once the angle and distance have
been calculated, they are send over I2C to the Arduino. The Arduino does not send any information back to the computer vision program.

The Raspberry Pi is not a fast computer, and takes about a second to detect a tag in a frame. It might be possible to optimize this code to get faster detection, but for 
now this performance is satifactory. This Pi continually looks for tags in frames from the camera. If it detectes a tag, it performs the angle and distance calculations. If theangle and distance are not the same as the last detected tags's it sends the information to the Arduino.

During the detection process the softawre takes into account the lense distortion of the camera. This is done by applying a set of distortion coefficients to the tag pose
detection. These coefficients were calculated in a calibration program and are unique to the camera used on this robot. Without the distortion coefficents, tags at the edge of
the camera's field of view would be warped, and the software would return innacurate angle and distance values.


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

To further simulate the controllers for Demo 2, each controller's reference response is compared to a faster response (faster speed) to ensure stability. Since the average time is a primary objective, the controllers are compared using a speed about twice as fast as is delivered to both motors. Since Derrick's target speed is approximately 5 feet in 10 seconds to approach the beacon, this speed is translated into a reasonable speed of approximately 2.2 rad/s for each motor. The angular velocities and position are compared for each motor. Figure # shows the Simulink block diagram for the faster speed to motor 1. A step response with twice the gain is added 10ms, or 1 sampling time duration, after the reference step response to account for the faster speed. The same procedure is used for motor 2.

![Motor1 block](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_block_speed.JPG)

Figure #: The controller block diagram for motor 1 with a faster speed at the reference input.

Using this block diagram, the controllers are simulated for the faster speed. Figure # shows the controller output with a faster speed for motor 1. Figure # shows the controller output for motor 2. Figure # shows the angular velocities for motor 1. Figure # shows the angular velocities for motor 2. As expected, the results are still reasonably stable and the rise time for each motor is under 0.3 seconds. 

![Motor1 controller fast](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_control.jpg)

Figure #: The P controller for faster speed for motor 1.

![Motor2 controller fast](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_control.jpg)

Figure #: The P controller for faster speed for motor 2.

![Motor1 velocity](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_veloc.jpg)

Figure #: The angular velocity for reference versus faster speed for motor 1.

![Motor2 velocity](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_veloc.jpg)

Figure #: The angular velocity for faster speed for motor 2.

Figure # shows the angular positions for motor 1. Figure # shows the angular positions for motor 2. The angular position diverges considerably as shown for each motor with a speed of 2.2 rad/s versus 0.5 rad/s reference, as expected. Thus, to reasonably reach a position of 5 feet in 10 seconds, each wheel moves approximately 22 radians in 10 seconds. If the motors respond exacty the same, then Derrick can reach his destination with a faster speed and stable output.

![Motor1 position speed](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_speed_pos.jpg)

Figure #: The angular position for faster speed for motor 1.

![Motor2 position speed](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor2_speed_pos.jpg)

Figure #: The angular position for faster speed for motor 2.

However, if one of the wheels accelerates faster than the other, this disturbance results in considerable overshoot. A step response 1.5 times faster than the reference is introduced into the system as a disturbance for motor 1. Figure # shows controller's lack of disturbance rejection, as no Ki or Kd gains are used for this system. Thus, to simulate the response, Ki gain is introduced and a PI controller is used for comparison. Figure # shows the Simulink block diagram with the disturbance and PI controller. The Ki gain is 17.4 V/(rad), and the Kp gain is tuned to 1.58 V/(rad/s). Figure # shows the system's ability to reject the disturbance given this Kp and Ki gain. 

![Motor1 disturb](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_disturb.JPG)

Figure #: The P controller response to a disturbance of acceleration for motor 1.

![Motor1 block disturb](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_disturb_block.JPG)

Figure #: The block diagram with a PI controller for an input disturbance for motor 1.

![Motor1 disturb reject](https://github.com/mtyler14/SEED_Group12/blob/master/Demo%202/images/motor1_disturb_reject.JPG)

Figure #: The PI controller response showing disturbance rejection for motor 1.

Though this works for the simulation, a Ki gain of 17.4 is not realistic for implementation. For the Arduino code, this unexpected acceleration of one of the motors is instead corrected for each mode of Derrick's motion. For example, if Derrick circles the beacon and the difference in the motors' speeds (in terms of counts) exceed 10, the controller outputs are scaled by 3.

Hence, the control system is tuned based on the encoder counts and angular velocity for each motor. The target position is reached with a final speed of 2.5 times the reference with minimal overshoot. The target objectives are successfully met with the necessary corrections to the speed based on the comparison of the motors' responses.
