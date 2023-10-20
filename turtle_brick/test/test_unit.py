"""Test turtle_twist operation"""

from turtle_brick.run_turtle import turtle_twist
from geometry_msgs.msg import Twist, Vector3

def test_turtle_twist():
    assert turtle_twist(1.0, 2.0, 3.0) == Twist(linear = Vector3(x = 1.0, y = 2.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 3.0))