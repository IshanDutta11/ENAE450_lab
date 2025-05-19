# Overview

This is the code used for our hardware competition trials. Read the code comments and report for a more thorough explanation. We determined that from our starting position, assuming an ideal orientation, we could go forward about 2 meters, turn about 80 degrees, then go forward until we exited the maze. However, hardcoding this would present some challenges. The first is that we wouldn’t know our starting orientation, and couldn’t be sure that we would need to turn the same amount at the 2-meter mark. The second issue was that turtlebot3 has the center of its drivetrain towards the front of the robot. That meant the high-level plan was to start from a random orientation, then: 

1. Determine the farthest point

2. Turn towards it

3.  Go forward until the robot is at risk of collision

4. Determine the farthest point

5. Turn towards it

6. Go forward indefinitely, the robot will exit the maze


# Functions overview

"init self" and "main" - Initializes the node, the lidar subscriber, and loops them. 

"move_robot" - Accepts linear and angular velocity and publishes them. 

"turn_robot" - Take the current heading, the desired heading, find the difference, and then turn at maximum speed for the necessary duration. 

"largest_i_over_3_1" and "largest_i_over_3_2" - Determines the "farthest" point and sets it as our desired heading. It determines this by taking the lidar values, cleaning them and narrowing the range, then finds the first (rightmost) value over 3 meters away. The reason these are two different functions is that they accept two different ranges. They realistically could be the same function, but the distinction was vestigial from an earlier iteration and we didn't want to change it if it wasn't broken. 

"control_loop" - Determines where in the 6 steps we are and what needs to be done/ what function to call at any given moment

"retrieve_distances" - Fills the lidar values and tests that they are valid. 