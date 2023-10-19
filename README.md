# ME495 Embedded Systems Homework 2
Author: Aditya Nair

This package contains a kinematic simulation of a turtle robot moving around an arena to catch a brick falling from the sky, in rviz. The turtle is holonomic and returns to the home position after catching a brick. The 2-Dimensional location of the robot is also emulated in a turtlesim node.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena and turtle simulation
2. Use `ros2 service call drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range
   `${embed video here}`

4. Here is a video of the turtle when the brick cannot be caught
   `${embed video here}`