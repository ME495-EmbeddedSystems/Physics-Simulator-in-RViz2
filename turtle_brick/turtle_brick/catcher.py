"""A basic tf2 listener that computes the distance and angle between the left and right frames."""
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Vector3, PoseStamped
import math

class Catcher(Node):
    """Listens to TF frames and logs information based on how they change."""

    def __init__(self):
        super().__init__("catcher")

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

        self.gravity = 9.8
        self.platform_height = 2.0
        self.predicted_flight_time = 0
        self.maxspeed = 5
        self.homeX = 5.5
        self.homeY = 5.5

        self.goal = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.updated = False

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

            if abs(self.height - self.prevHeight) > 0.0 and abs(self.height - self.prevHeight) < self.movement_tol and not self.updated:

                # self.get_logger().info(f"Brick is falling")

                self.targetX = trans.transform.translation.x
                self.targetY = trans.transform.translation.y

                self.predicted_flight_time = (2 * (self.height - self.platform_height) / self.gravity) ** 0.5

                self.speed_req = math.dist([self.targetX, self.targetY], [self.homeX, self.homeY]) / self.predicted_flight_time

                if self.maxspeed >= self.speed_req:

                    self.get_logger().info("Reachable")

                    goalpose = PoseStamped()
                    goalpose.pose.position.x = self.targetX
                    goalpose.pose.position.y = self.targetY
                    self.goal.publish(goalpose)
                    self.updated = True

                else:

                    self.get_logger().info("Unreachable")
                    goalpose = PoseStamped()
                    goalpose.pose.position.x = self.homeX
                    goalpose.pose.position.y = self.homeY
                    self.goal.publish(goalpose)
                    self.updated = True


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