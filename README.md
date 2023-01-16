# robot_2R_serial
Arduino programs for forward and inverse kinematics of a two link planar serial robot.

1) Forward kinematics
Input: angles, Output: x,y positions.
The code writes the input angles to two (3-pin) servo motors.

2) Inverse kinematics:
Input: x,y positions, Output: two solutions.
The code writes the angles got from IK solution to the servo motors.
Video: https://youtube.com/shorts/0Z6gWwjv58g?feature=share

3) Toggle between positions:
Input angles corresponding to two positions and the robot will toggle
between them. 
Video: https://youtube.com/shorts/MVV_LeegjDw?feature=share
