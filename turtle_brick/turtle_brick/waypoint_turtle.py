import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import pi
from .quaternion import angle_axis_to_quaternion
from sensor_msgs.msg import JointState

import math


class WaypointTurtle(Node):
    """
    Moves some frames around.

    Static Broadcasts:
       world -> base
    Broadcasts:
       base -> left and base -> right
    """

    def __init__(self):
        super().__init__('waypoint_turtle')
        # Static broadcasters publish on /tf_static. We will only need to publish this once
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Now create the transform, noted that it must have a parent frame and a timestamp
        # The header contains the timing information and frame id
        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"

        # The base frame will be raised in the z direction by 1 meter
        # and be aligned with world We are relying on the default values
        # of the transform message (which defaults to no rotation)
        world_odom_tf.transform.translation.z = 0.0
        self.static_broadcaster.sendTransform(world_odom_tf)
        self.get_logger().info("Static Transform: world->odom")

        # TO DO
        self.wheel_radius = 0.2 * 1.618
        self.targetX = 4.0
        self.targetY = 5.0

        # Robot configuration
        self.wheel_pos = 0
        self.tip_pos = 0
        self.tip = True
        self.x = 0
        self.y = 0
        self.theta = math.pi/4

        # Velocities
        self.wheel_omg = 0.8  # used to control frame movement
        self.swivel_omg = 0.8  # used to control frame movement
        self.vx = self.wheel_radius * self.wheel_omg * math.cos(self.theta)
        self.vy = self.wheel_radius * self.wheel_omg * math.sin(self.theta)

        # create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        # Create a timer to do the rest of the transforms
        self.frequency = 100.0
        self.dt = 1/self.frequency
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # Commanded joint positions
        self.positioncommander = self.create_publisher(JointState, '/cmd_joints', 10)

    def timer_callback(self):

        # Calculate various relative positions
        self.wheel_pos = float(self.wheel_pos + self.wheel_omg * self.dt)
        self.wheel_pos = float(self.wheel_pos + self.wheel_omg * self.dt)

        self.tip_pos = -0.187 if self.tip else 0

        # self.get_logger().info(f"Wheel position: {self.wheel_pos}, Swivel position: {self.theta}, Tip: {self.tip}")

        self.x = self.x + self.vx * self.dt
        self.y = self.y + self.vy * self.dt
        # self.theta = math.atan2(math.sin(self.theta + self.swivel_omg * self.dt), math.cos(self.theta + self.swivel_omg * self.dt))

        self.get_logger().info(f"X: {self.x}, Y: {self.y}, theta {math.degrees(self.theta)}")

        # Update velocities
        self.theta = math.atan2(self.targetY - self.y, self.targetX - self.x)
        self.vx = self.wheel_radius * self.wheel_omg * math.cos(self.theta)
        self.vy = self.wheel_radius * self.wheel_omg * math.sin(self.theta)

        # Format and publish joint state messages
        joints_states_msg = JointState()
        joints_states_msg.header.stamp = self.get_clock().now().to_msg()
        joints_states_msg.name = ['wheel_axle','swivel','tip']
        joints_states_msg.position = [self.wheel_pos, self.theta, self.tip_pos]

        self.positioncommander.publish(joints_states_msg)

        odom_base = TransformStamped()
        odom_base.header.frame_id = "odom"
        odom_base.child_frame_id = "base_link"
        odom_base.transform.translation.x = float(self.x)
        odom_base.transform.translation.y = float(self.y)
        # get a quaternion corresponding to a rotation by theta about an axis
        degrees = float(self.theta)
        radians = degrees * pi / 180.0
        odom_base.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, 1])

        time = self.get_clock().now().to_msg()
        odom_base.header.stamp = time

        self.broadcaster.sendTransform(odom_base)

        # base_right = TransformStamped()
        # base_right.header.frame_id = "base"
        # base_right.child_frame_id = "right"
        # base_right.transform.translation.x = float(self.dx)
        # base_right.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])

        # # don't forget to put a timestamp
        # time = self.get_clock().now().to_msg()
        # base_right.header.stamp = time
        # base_left.header.stamp = time

        # self.broadcaster.sendTransform(base_left)
        # self.broadcaster.sendTransform(base_right)

        # # update the movement
        # self.dx -= 1
        # if self.dx == 0:
            # self.dx = 10


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTurtle()
    rclpy.spin(node)
    rclpy.shutdown()
