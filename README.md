# ME495 Embedded Systems Homework 2
Author: Aditya Nair

This package contains a kinematic simulation of a turtle robot moving around an arena to catch a brick falling from the sky, in rviz. The turtle is holonomic and returns to the home position after catching a brick. The 2-Dimensional location of the robot is also emulated in a turtlesim node.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena and turtle simulation
2. Use `ros2 service call drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range

https://private-user-images.githubusercontent.com/59332714/276552837-f54f96b2-79ea-443a-9939-add791b4faa0.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTEiLCJleHAiOjE2OTc3MDY4NDIsIm5iZiI6MTY5NzcwNjU0MiwicGF0aCI6Ii81OTMzMjcxNC8yNzY1NTI4MzctZjU0Zjk2YjItNzllYS00NDNhLTk5MzktYWRkNzkxYjRmYWEwLndlYm0_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBSVdOSllBWDRDU1ZFSDUzQSUyRjIwMjMxMDE5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDIzMTAxOVQwOTA5MDJaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT01MjFhNjQwODUwNGQ1MmUxNTE2MjA3OTkxZTRjNjFiOTQwNTA1NmFjZDg4Zjc2ZDFmMmNmMWUzNjM2MDZiYmRlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.ARaeBJZxiDORq8HAAcXN4soS2z-YtpUY3rRVm704zTE

4. Here is a video of the turtle when the brick cannot be caught
   `${embed video here}`
