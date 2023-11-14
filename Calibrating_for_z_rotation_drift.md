
# How to calibrate the rotation around z axis adjustment for x and y degrees

1. Set adjustment to zero and flash the robot with it.

2. Connect robot to remote. Start rotating the robot. Not which direction robot leans to with what direction joystick is telling it to turn to.

3. Set a adjustment value that is not too high(1-20) as a starting point and test the robot turning again.

4. If the drift got worse switch the sign (positive/negative) of the adjustment to the opposite.

5. Keep trying values progressively getting bigger until you see that the sides where the robot leaned to has switched. 

6. Roll back the adjustment until the robot switched back to its original side. This way you know the range where your adjustment needs to be.

7. Keep trying values in the range by increasing or decreasing by 0.5. 

Note different values can behave differently. My robot behaved very good from 14-20 and got worse bellow 14 but never just switched direction barely. It can also be that turning to some direction causes more drift or some angle of the yaw produces a drift to one direction specifically.