```
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch maze_simulation maze_0_inside.launch.py

ros2 run maze_simulation simulation_node

Maze 0 results: 44.8s
Maze 1 results: 1m 19.62s
Maze 2 results: 2m 4.8s
```

## 90 Degree Turn
An extremely useful tool to navigate the maze would be to have a reliable 90 degree turn. In simulation we found that it was impossible to turn using time delays. The robot would overshoot heavily, which may be a result of the performance of the machine using it. Regardless, it was not reliable, and instead we had to shift our focus to just using obstacle avoidance.

## Forward Field of View with correcting
First thing we had to do was analyze the distance in front of the robot. We decided not to use just one value since the robot would not be able to correct itself as it was going forward since it would be unaware of its front two corners. Additionally, to help it correct itself, if the walls exceeded a certain distance from the robot, it would begin turning as well. We retrieved a range of left and right distances and averaged them to use for this condition.


## Turning
If the robot’s front and sides are too close to the walls, it would begin turning towards whichever side had the largest distance at a rate of about π/2 rad/s. It would begin to move forward again once the distance in front of the robot was large enough. That is essentially the control loop. It would stop the control loop, spin, and rest in place once the left and right values reached infinity, indicating that we were out of the maze.

