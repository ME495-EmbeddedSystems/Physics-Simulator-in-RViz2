"""Draw robotic grippers using RVIZ markers and make them interactive. """


import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl, MarkerArray
from .quaternion import angle_axis_to_quaternion
from turtle_brick_interfaces.srv import Place
from enum import Enum, auto
from std_srvs.srv import Empty

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration.
    """
    INITIAL = auto(),
    BRICK_PLACED = auto(),
    BRICK_DROPPED = auto()

class Arena(Node):
    """
    PUBLISHES:
    visualization_marker (visualization_messages/msg/Marker) - The markers that we are drawing

    SUBSCRIBES:
    Subscribes: to an interactive marker server
    """
    def __init__(self):
        super().__init__("arena")

        self.brick_height = 11.0
        self.brick_heightvel = 0
        self.brick_x = 0
        self.brick_y = 0
        self.brick_size = 0.5
        self.gravity = 9.8
        self.flight_time = 0
        self.predicted_flight_time = (2 * self.brick_height / self.gravity) ** 0.5
        self.platform_height = 2

        self.arena_breadth = 11.0
        self.arena_length = 11.0
        self.wall_height = 1.6
        self.wall_width = 0.5

        self.state = State.INITIAL

        # Broadcast brick frame
        self.broadcaster = TransformBroadcaster(self)
        world_brick_tf = TransformStamped()
        world_brick_tf.header.stamp = self.get_clock().now().to_msg()
        world_brick_tf.header.frame_id = "world"
        world_brick_tf.child_frame_id = "brick"
        world_brick_tf.transform.translation.z = self.brick_height
        self.broadcaster.sendTransform(world_brick_tf)

        # We use TRANSIENT_LOCAL durability for the publisher. By setting both publisher and subscriber
        # Durability to TRANSIENT_LOCAL we emulate the effect of "latched publishers" from ROS 1
        # (See https://github.com/ros2/ros2/issues/464)
        # Essentially this means that when subscribers first connect to the topic they receive the
        # last message published on the topic. Useful for example because rviz might open after
        # the initial markers are published
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub1 = self.create_publisher(Marker, "visualization_marker", markerQoS)
        self.ma_pub = self.create_publisher(MarkerArray, "visualization_marker_array", markerQoS)
        # The second link is oriented at 90 degrees
        self.m = Marker()
        self.m.header.frame_id = "brick"
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 1
        self.m.type = Marker.CUBE
        self.m.action = Marker.ADD
        self.m.scale.x = 1.618 * self.brick_size
        self.m.scale.y = 1.618 * 1.618 * self.brick_size
        self.m.scale.z = 1.0 * self.brick_size
        self.m.pose.position.x = 0.0
        self.m.pose.position.y = 0.0
        self.m.pose.position.z = 0.0
        self.m.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        # self.m.pose.orientation.y = 0.0
        # self.m.pose.orientation.z = 0.0
        # self.m.pose.orientation.w = .707
        self.m.color.r = 0.8
        self.m.color.g = 0.8
        self.m.color.b = 0.8
        self.m.color.a = 1.0
        self.pub1.publish(self.m)

        self.walls = MarkerArray()

        # North Wall
        self.wall_north = Marker()
        self.wall_north.header.frame_id = "world"
        self.wall_north.header.stamp = self.get_clock().now().to_msg()
        self.wall_north.id = 2
        self.wall_north.type = Marker.CUBE
        self.wall_north.action = Marker.ADD
        self.wall_north.scale.x = self.arena_length
        self.wall_north.scale.y = self.wall_width
        self.wall_north.scale.z = self.wall_height
        self.wall_north.pose.position.x = self.arena_length / 2
        self.wall_north.pose.position.y = self.arena_breadth + self.wall_width / 2
        self.wall_north.pose.position.z = self.wall_height / 2
        self.wall_north.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        # self.m.pose.orientation.y = 0.0
        # self.m.pose.orientation.z = 0.0
        # self.m.pose.orientation.w = .707
        self.wall_north.color.r = 0.8
        self.wall_north.color.g = 0.8
        self.wall_north.color.b = 0.8
        self.wall_north.color.a = 1.0

        self.walls.markers.append(self.wall_north)

        # South Wall
        self.wall_south = Marker()
        self.wall_south.header.frame_id = "world"
        self.wall_south.header.stamp = self.get_clock().now().to_msg()
        self.wall_south.id = 3
        self.wall_south.type = Marker.CUBE
        self.wall_south.action = Marker.ADD
        self.wall_south.scale.x = self.arena_length
        self.wall_south.scale.y = self.wall_width
        self.wall_south.scale.z = self.wall_height
        self.wall_south.pose.position.x = self.arena_length / 2
        self.wall_south.pose.position.y = -self.wall_width / 2
        self.wall_south.pose.position.z = self.wall_height / 2
        self.wall_south.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        # self.m.pose.orientation.y = 0.0
        # self.m.pose.orientation.z = 0.0
        # self.m.pose.orientation.w = .707
        self.wall_south.color.r = 0.8
        self.wall_south.color.g = 0.8
        self.wall_south.color.b = 0.8
        self.wall_south.color.a = 1.0

        self.walls.markers.append(self.wall_south)

        # West Wall
        self.wall_west = Marker()
        self.wall_west.header.frame_id = "world"
        self.wall_west.header.stamp = self.get_clock().now().to_msg()
        self.wall_west.id = 4
        self.wall_west.type = Marker.CUBE
        self.wall_west.action = Marker.ADD
        self.wall_west.scale.x = self.wall_width
        self.wall_west.scale.y = self.arena_breadth
        self.wall_west.scale.z = self.wall_height
        self.wall_west.pose.position.x = -self.wall_width / 2
        self.wall_west.pose.position.y = self.arena_breadth / 2
        self.wall_west.pose.position.z = self.wall_height / 2
        self.wall_west.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        # self.m.pose.orientation.y = 0.0
        # self.m.pose.orientation.z = 0.0
        # self.m.pose.orientation.w = .707
        self.wall_west.color.r = 0.8
        self.wall_west.color.g = 0.8
        self.wall_west.color.b = 0.8
        self.wall_west.color.a = 1.0

        self.walls.markers.append(self.wall_west)

        # East Wall
        self.wall_east = Marker()
        self.wall_east.header.frame_id = "world"
        self.wall_east.header.stamp = self.get_clock().now().to_msg()
        self.wall_east.id = 5
        self.wall_east.type = Marker.CUBE
        self.wall_east.action = Marker.ADD
        self.wall_east.scale.x = self.wall_width
        self.wall_east.scale.y = self.arena_breadth
        self.wall_east.scale.z = self.wall_height
        self.wall_east.pose.position.x = self.arena_length + self.wall_width / 2
        self.wall_east.pose.position.y = self.arena_breadth / 2
        self.wall_east.pose.position.z = self.wall_height / 2
        self.wall_east.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        # self.m.pose.orientation.y = 0.0
        # self.m.pose.orientation.z = 0.0
        # self.m.pose.orientation.w = .707
        self.wall_east.color.r = 0.8
        self.wall_east.color.g = 0.8
        self.wall_east.color.b = 0.8
        self.wall_east.color.a = 1.0

        self.walls.markers.append(self.wall_east)

        self.pub1.publish(self.m)
        self.ma_pub.publish(self.walls)

        # self.m1 = Marker()
        # self.m1.header.frame_id = "brick"
        # self.m1.header.stamp = self.get_clock().now().to_msg()
        # self.m1.id = 2
        # self.m1.type = Marker.CUBE
        # self.m1.action = Marker.ADD
        # self.m1.scale.x = 1.0
        # self.m1.scale.y = 1.0
        # self.m1.scale.z = 3.0
        # self.m1.pose.position.x = -5.0
        # self.m1.pose.position.y = 2.0
        # self.m1.pose.position.z = -1.0
        # self.m1.pose.orientation.x = .707
        # self.m1.pose.orientation.y = 0.0
        # self.m1.pose.orientation.z = 0.0
        # self.m1.pose.orientation.w = .707
        # self.m1.color.r = 1.0
        # self.m1.color.g = 0.0
        # self.m1.color.b = 0.0
        # self.m1.color.a = 1.0
        # self.pub1.publish(self.m1)

        # self.server = InteractiveMarkerServer(self, "arena_marker")

        # int_marker = InteractiveMarker()
        # int_marker.header.frame_id = "brick"
        # int_marker.name = "gripper"
        # int_marker.description = "Move to open/close the gripper"
        # int_marker.pose.orientation.w = .707
        # int_marker.pose.orientation.z = .707

        # box_marker = Marker()
        # box_marker.type = Marker.SPHERE
        # box_marker.scale.x = 0.5
        # box_marker.scale.y = 0.5
        # box_marker.scale.z = 0.5
        # box_marker.color.r = 1.0
        # box_marker.color.g = 0.0
        # box_marker.color.b = 0.0
        # box_marker.color.a = 1.0

        # box_control = InteractiveMarkerControl()
        # box_control.always_visible = True
        # box_control.markers.append(box_marker)


        # int_marker.controls.append(box_control)

        # speed_control = InteractiveMarkerControl()
        # speed_control.name = "move"
        # speed_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        # int_marker.controls.append(speed_control)

        # self.server.insert(int_marker, feedback_callback=self.callback)
        # self.server.applyChanges()

        self.frequency = 250.0
        self.dt = 1/self.frequency
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        self.place = self.create_service(Place, "place", self.place_callback)
        self.drop = self.create_service(Empty, "drop", self.drop_callback)

    def callback(self, feedback):
        """ Callback for interactive markers.  feedback contains the pose of the marker from rviz """
        self.get_logger().info("Log")
        w = feedback.pose.orientation.w
        self.m.pose.position.x = 3*w
        # self.m1.pose.position.x = -self.m.pose.position.x
        self.m.action = Marker.MODIFY
        # self.m1.action = Marker.MODIFY
        self.m.header.stamp = self.get_clock().now().to_msg()
        # self.m1.header.stamp = self.get_clock().now().to_msg()
        self.pub1.publish(self.m)
        # self.pub1.publish(self.m1)
        self.get_logger().info("Logout")

    def timer_callback(self):

        time = self.get_clock().now().to_msg()

        if self.state == State.INITIAL:

            self.get_logger().info("Initial State")

        elif self.state == State.BRICK_PLACED:

            self.get_logger().info("Placed State")

            world_brick_tf = TransformStamped()
            world_brick_tf.header.stamp = time
            world_brick_tf.header.frame_id = "world"
            world_brick_tf.child_frame_id = "brick"
            world_brick_tf.transform.translation.x = self.brick_x
            world_brick_tf.transform.translation.y = self.brick_y
            world_brick_tf.transform.translation.z = self.brick_height
            self.broadcaster.sendTransform(world_brick_tf)

            self.m.action = Marker.MODIFY
            self.m.header.stamp = time
            self.pub1.publish(self.m)

            # self.walls.action = Marker.MODIFY
            # self.walls.header.stamp = time
            # self.ma_pub.publish(self.walls)

        elif self.state == State.BRICK_DROPPED:

            self.get_logger().info("Dropped State")

            self.brick_height = self.brick_height + self.brick_heightvel * self.dt
            self.brick_heightvel = self.brick_heightvel - self.gravity * self.dt

            self.flight_time = self.flight_time + self.dt

            self.get_logger().info(f"{self.brick_height} -- {self.flight_time} -- {self.predicted_flight_time}")

            if self.brick_height <= self.platform_height:

                self.brick_height = self.platform_height
                self.state = State.BRICK_CAUGHT

            elif self.self.brick_height <= 0:

                self.brick_height = 0
                self.state = State.BRICK_FALLEN

            world_brick_tf = TransformStamped()
            world_brick_tf.header.stamp = time
            world_brick_tf.header.frame_id = "world"
            world_brick_tf.child_frame_id = "brick"
            world_brick_tf.transform.translation.x = self.brick_x
            world_brick_tf.transform.translation.y = self.brick_y
            world_brick_tf.transform.translation.z = self.brick_height
            self.broadcaster.sendTransform(world_brick_tf)

            self.m.action = Marker.MODIFY
            self.m.header.stamp = time
            self.pub1.publish(self.m)

            # self.walls.action = Marker.MODIFY
            # self.walls.header.stamp = time
            # self.ma_pub.publish(self.walls)

        elif self.state == State.BRICK_CAUGHT:

            self.get_logger().info("Caught State")

            world_brick_tf = TransformStamped()
            world_brick_tf.header.stamp = time
            world_brick_tf.header.frame_id = "world"
            world_brick_tf.child_frame_id = "brick"
            world_brick_tf.transform.translation.x = self.brick_x
            world_brick_tf.transform.translation.y = self.brick_y
            world_brick_tf.transform.translation.z = self.brick_height
            self.broadcaster.sendTransform(world_brick_tf)

            self.m.action = Marker.MODIFY
            self.m.header.stamp = time
            self.pub1.publish(self.m)

        elif self.state == State.BRICK_FALLEN:

            self.get_logger().info("Fallen State")

            world_brick_tf = TransformStamped()
            world_brick_tf.header.stamp = time
            world_brick_tf.header.frame_id = "world"
            world_brick_tf.child_frame_id = "brick"
            world_brick_tf.transform.translation.x = self.brick_x
            world_brick_tf.transform.translation.y = self.brick_y
            world_brick_tf.transform.translation.z = self.brick_height
            self.broadcaster.sendTransform(world_brick_tf)

            self.m.action = Marker.MODIFY
            self.m.header.stamp = time
            self.pub1.publish(self.m)



    def place_callback(self, request, response):

        self.get_logger().info("Placing Brick")

        time = self.get_clock().now().to_msg()

        self.brick_x = request.x
        self.brick_y = request.y

        world_brick_tf = TransformStamped()
        world_brick_tf.header.stamp = self.get_clock().now().to_msg()
        world_brick_tf.header.frame_id = "world"
        world_brick_tf.child_frame_id = "brick"
        world_brick_tf.transform.translation.x = request.x
        world_brick_tf.transform.translation.y = request.y
        world_brick_tf.transform.translation.z = self.brick_height
        self.broadcaster.sendTransform(world_brick_tf)

        self.m.action = Marker.MODIFY
        self.m.header.stamp = time
        self.pub1.publish(self.m)

        # self.walls.markers.action = Marker.MODIFY
        # self.walls.header.stamp = time
        # self.ma_pub.publish(self.walls)

        self.state = State.BRICK_PLACED

        return response

    def drop_callback(self, request, response):

        self.get_logger().info("Dropping Brick")

        self.state = State.BRICK_DROPPED

        return response



def arena(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()