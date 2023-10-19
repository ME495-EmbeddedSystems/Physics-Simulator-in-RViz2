"""A basic tf2 listener that computes the distance and angle between the left and right frames."""
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, InteractiveMarkerControl, MarkerArray
from geometry_msgs.msg import TransformStamped, Twist, Vector3, PoseStamped
from turtle_brick_interfaces.msg import Tilt
import math
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from .quaternion import angle_axis_to_quaternion


class Catcher(Node):
    """Listens to TF frames and logs information based on how they change."""

    def __init__(self):
        super().__init__("catcher")

        self.declare_parameter("platform_height", 1.5,
                               ParameterDescriptor(description="Height of robot"))
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        
        self.declare_parameter("wheel_radius", 0.2,
                               ParameterDescriptor(description="Wheel radius"))
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value

        self.declare_parameter("max_velocity", 5.0,
                               ParameterDescriptor(description="Maximum translational speed of robot"))
        self.speed = self.get_parameter("max_velocity").get_parameter_value().double_value

        self.declare_parameter("gravity_accel", 9.8,
                               ParameterDescriptor(description="Positive acceleration due to gravity"))
        self.gravity = self.get_parameter("gravity_accel").get_parameter_value().double_value

        # The buffer stores received tf frames
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.frequency = 250.0
        self.dt = 1 / self.frequency

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.prevHeight = 0
        self.height = 0
        self.movement_tol = 0.1

        self.empty = 0

        self.predicted_flight_time = 0
        self.homeX = 5.5
        self.homeY = 5.5

        self.goal = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.tilt = self.create_publisher(Tilt, "tilt", 10)
        self.setangle = -0.1807

        self.state = 1
        self.dtol = 0.05

        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.um_pub = self.create_publisher(Marker, "unreachable", markerQoS)
        self.um = Marker()
        self.um.header.frame_id = "world"
        self.um.frame_locked = True
        self.um.lifetime.sec = 3
        self.um.header.stamp = self.get_clock().now().to_msg()
        # self.um.description = "UNREACHABLE"
        self.um.id = 8
        self.um.type = Marker.TEXT_VIEW_FACING
        self.um.text = "UNREACHABLE"
        self.um.action = Marker.ADD
        # self.um.scale.x = 1.618 * self.brick_size
        # self.um.scale.y = 1.618 * 1.618 * self.brick_size
        self.um.scale.z = 3.0
        self.um.pose.position.x = 0.0
        self.um.pose.position.y = 0.0
        self.um.pose.position.z = 2.0
        # self.um.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        self.um.color.r = 1.0
        self.um.color.g = 1.0
        self.um.color.b = 1.0
        self.um.color.a = 1.0

    def timer_callback(self):
        # we listen in a try block.  If a frame has not been published
        # recently enough, then there will be an error and we continue.
        # For demonstration purposes we catch each exception individually
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            trans = self.buffer.lookup_transform("world", "brick", rclpy.time.Time())
            # self.get_logger().info(f"Height is: {trans.transform.translation.z}")
            self.height = trans.transform.translation.z

            # self.get_logger().info(f"{self.prevHeight}, {self.height}")
            # self.get_logger().info(f"{self.prevHeight - self.height}")

            if abs(self.height - self.prevHeight) > 0.0 and abs(self.height - self.prevHeight) < self.movement_tol and self.state == 1:

                self.get_logger().info(f"Brick is falling")

                self.targetX = trans.transform.translation.x
                self.targetY = trans.transform.translation.y

                self.predicted_flight_time = (2 * (self.height - self.platform_height) / self.gravity) ** 0.5

                self.speed_req = math.dist([self.targetX, self.targetY], [self.homeX, self.homeY]) / self.predicted_flight_time

                if self.speed >= self.speed_req:

                    self.get_logger().info("Reachable")

                    goalpose = PoseStamped()
                    goalpose.pose.position.x = self.targetX
                    goalpose.pose.position.y = self.targetY
                    self.goal.publish(goalpose)
                    self.state = 2

                else:

                    self.get_logger().info("Unreachable")
                    goalpose = PoseStamped()
                    goalpose.pose.position.x = self.homeX
                    goalpose.pose.position.y = self.homeY
                    self.goal.publish(goalpose)
                    self.um_pub.publish(self.um)
                    self.state = 5


            if self.height <= self.platform_height and self.state == 2:

                self.get_logger().info("Caught")
                goalpose = PoseStamped()
                goalpose.pose.position.x = self.homeX
                goalpose.pose.position.y = self.homeY
                self.goal.publish(goalpose)
                self.state = 3

            if math.dist([trans.transform.translation.x, trans.transform.translation.y], [self.homeX, self.homeY]) <= self.dtol and self.state == 3:

                self.tiltang = Tilt()
                self.tiltang.tilt_angle = self.setangle
                self.tilt.publish(self.tiltang)
                
                self.get_logger().info("Traveled back")
                self.state = 4

            if self.state == 5:

                self.get_logger().info("")

            self.prevHeight = self.height

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            # self.get_logger().info(f"")
            self.empty = 0
            # self.get_logger().info(f"Lookup exception: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()