"""Draw walls and brick that constitute the arena and govern physics of the brick. """

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
import math
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration.
    """
    INITIAL = auto(),
    BRICK_PLACED = auto(),
    BRICK_DROPPED = auto(),
    BRICK_CAUGHT = auto(),
    BRICK_FALLEN = auto(),
    BRICK_SLIDE = auto(),
    BRICK_FROZEN = auto()

class Arena(Node):
    """ 
    Renders the walls of the arena as a marker array, renders the brick as a marker and simulates its physics.

    PUBLISHES:
        visualization_marker (visualization_messages/msg/Marker) : A Marker representing the brick.
        visualization_marker_array (visualization_messages/msg/MarkerArray) : A Marker representing the arena walls.

    SERVICES:
        place : places brick at desired location.
        drop : drops brick from placed location.

    """
    def __init__(self):
        super().__init__("arena")

        # Retreive parametrs defined in config/turtle.yaml.

        # Height of the robot
        self.declare_parameter("platform_height", 1.5,
                               ParameterDescriptor(description="Height of robot"))
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        
        # Wheel radius
        self.declare_parameter("wheel_radius", 0.2,
                               ParameterDescriptor(description="Wheel radius"))
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value

        # Maximum translational speed of robot
        self.declare_parameter("max_velocity", 5.0,
                               ParameterDescriptor(description="Maximum translational speed of robot"))
        self.speed = self.get_parameter("max_velocity").get_parameter_value().double_value

        # Downwards acceleration due to gravity
        self.declare_parameter("gravity_accel", 9.8,
                               ParameterDescriptor(description="Positive acceleration due to gravity"))
        self.gravity = self.get_parameter("gravity_accel").get_parameter_value().double_value


        # Initialize brick variables.
        self.brick_height = 6.0 # Height from ground till base of the brick
        self.brick_x = 0 # X-Position of brick in world frame
        self.brick_y = 0 # Y-POsition of brick in world frame
        self.brick_size = 0.2 # Size of brick (Z-axis)
        self.brick_heightvel = 0 # Inital downward velocity of brick
        self.predicted_flight_time = (2 * (self.brick_height - self.platform_height) / self.gravity) ** 0.5 # Time taken to fall to the ground

        self.arena_breadth = 11.0 # Breadth of arena (Along Y-axis)
        self.arena_length = 11.0 # Breadth of arena (Along Z-axis)
        self.wall_height = 0.5 # Height of walls
        self.wall_width = 0.1 # Width of walls
        self.homeX = 5.5 # X-position world->odom
        self.homeY = 5.5 # Y-position world->odom
        self.dtol = 0.05 # Distance tolerance for assessing state of the brick
        self.tilt_angle = -0.1807 # Angle at which the brick slides off
        self.platform_radius = 0.3 # Radius of turtle robot platform
        self.unreachable = False # True if robot cannot reach the brick

        self.state = State.INITIAL # State of brick before it is placed

        # Create broadcaster for transform world->brick.
        self.broadcaster = TransformBroadcaster(self)
        
        # Broadcast brick frame.
        world_brick_tf = TransformStamped()
        world_brick_tf.header.stamp = self.get_clock().now().to_msg()
        world_brick_tf.header.frame_id = "world"
        world_brick_tf.child_frame_id = "brick"
        world_brick_tf.transform.translation.z = self.brick_height
        self.broadcaster.sendTransform(world_brick_tf)

        # Initialize Quality of Service.
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers

        # Create brick marker publisher.
        self.pub1 = self.create_publisher(Marker, "visualization_marker", markerQoS)

        # Create wall publisher.
        self.ma_pub = self.create_publisher(MarkerArray, "visualization_marker_array", markerQoS)

        # Markers
        
        # Specify brick marker parameters.
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
        self.m.color.r = 0.7
        self.m.color.g = 0.05
        self.m.color.b = 0.05
        self.m.color.a = 1.0

        # Create marker array for four walls.
        self.walls = MarkerArray()

        # North Wall.
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
        self.wall_north.color.r = 0.8
        self.wall_north.color.g = 0.05
        self.wall_north.color.b = 0.05
        self.wall_north.color.a = 1.0

        self.walls.markers.append(self.wall_north)

        # South Wall.
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
        self.wall_south.color.r = 0.8
        self.wall_south.color.g = 0.05
        self.wall_south.color.b = 0.05
        self.wall_south.color.a = 1.0

        self.walls.markers.append(self.wall_south)

        # West Wall.
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
        self.wall_west.color.r = 0.8
        self.wall_west.color.g = 0.05
        self.wall_west.color.b = 0.05
        self.wall_west.color.a = 1.0

        self.walls.markers.append(self.wall_west)

        # East Wall.
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
        self.wall_east.color.r = 0.8
        self.wall_east.color.g = 0.05
        self.wall_east.color.b = 0.05
        self.wall_east.color.a = 1.0

        self.walls.markers.append(self.wall_east)

        # Publish brick marker.
        self.pub1.publish(self.m)
        
        # Publish walls marker array.
        self.ma_pub.publish(self.walls)

        # Create timer.
        self.frequency = 250.0 # Frequency 
        self.dt = 1/self.frequency # Timestep
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # Services

        # Places brick at desired location.
        self.place = self.create_service(Place, "place", self.place_callback)

        # Drops brick from placed position.
        self.drop = self.create_service(Empty, "drop", self.drop_callback)

    def timer_callback(self):

        """ Timer Callback for this node. Dictates motion of brick based on initialized parameters.
        """

        # Current time for every iteration.
        time = self.get_clock().now().to_msg()

        # Before the brick is dropped, do nothing.
        if self.state == State.INITIAL:

            pass

        # After brick is placed, broadcast transform and publish marker.
        elif self.state == State.BRICK_PLACED:

            # Broadcast world->brick transform and publish brick marker.
            
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

        # When brick is falling, check if the brick is caught or if it's fallen on the ground.
        elif self.state == State.BRICK_DROPPED:

            # Decide whether robot can reach the brick.
            
            # Speed required to reach the brick.
            speed_req = math.dist([self.brick_x, self.brick_y], [self.homeX, self.homeY]) / self.predicted_flight_time

            if self.speed >= speed_req:

                self.unreachable = False
            
            else:

                self.unreachable = True

            # Update height of falling brick.
            self.brick_height = self.brick_height + self.brick_heightvel * self.dt
            self.brick_heightvel = self.brick_heightvel - self.gravity * self.dt

            # If the robot can reach the brick, make brick stop at robot height.
            if self.brick_height <= self.platform_height and not self.unreachable:

                self.brick_height = self.platform_height
                self.brick_heightvel = 0.0
                self.state = State.BRICK_CAUGHT
                self.get_logger().info("Caught Brick")


            # If robot can't reach the brick, make brick stop at ground.
            elif self.brick_height <= 0.0 and self.unreachable:

                self.brick_height = 0.0
                self.state = State.BRICK_FALLEN
                self.get_logger().info("Brick Fell :(")

            # Broadcast world->brick transform and publish brick marker.
            
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

        elif self.state == State.BRICK_CAUGHT:
            
            self.brick_vx = self.speed * math.cos(math.atan2(self.homeY - self.brick_y, self.homeX - self.brick_x))
            self.brick_vy = self.speed * math.sin(math.atan2(self.homeY - self.brick_y, self.homeX - self.brick_x))

            self.brick_x = self.brick_x + self.brick_vx * self.dt
            self.brick_y = self.brick_y + self.brick_vy * self.dt

            if math.dist([self.brick_x,self.brick_y],[self.homeX,self.homeY]) <= self.dtol:

                self.brick_vx = 0
                self.brick_vy = 0
                self.state = State.BRICK_SLIDE
                self.get_logger().info("Brick is ready to dump")

            
            # Broadcast world->brick transform and publish brick marker.
            
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

        # When robot reaches home and tilts platform, make brick slide.
        elif self.state == State.BRICK_SLIDE:

            # Update X and Z velocities according to sliding physics.
            self.brick_vx = self.brick_vx + math.cos(self.tilt_angle) * math.sin(self.tilt_angle) * self.gravity * self.dt
            self.brick_heightvel = self.brick_heightvel - math.sin(self.tilt_angle)**2 * self.gravity * self.dt

            # Update X and Z position.
            self.brick_x = self.brick_x + self.brick_vx * self.dt
            self.brick_height = self.brick_height + self.brick_heightvel * self.dt

            # Make brick freeze in air, after it slides off the platform.
            if math.dist([self.brick_x,self.brick_y],[self.homeX,self.homeY]) > 2 * self.platform_radius:

                self.get_logger().info("Brick Dumped")
                self.state = State.BRICK_FROZEN

            # Broadcast world->brick transform and publish brick marker.
            
            world_brick_tf = TransformStamped()
            world_brick_tf.header.stamp = time
            world_brick_tf.header.frame_id = "world"
            world_brick_tf.child_frame_id = "brick"
            world_brick_tf.transform.translation.x = self.brick_x
            world_brick_tf.transform.translation.y = self.brick_y
            world_brick_tf.transform.translation.z = self.brick_height
            world_brick_tf.transform.rotation = angle_axis_to_quaternion(self.tilt_angle, [0, 1, 0])
            self.broadcaster.sendTransform(world_brick_tf)
            self.m.action = Marker.MODIFY
            self.m.header.stamp = time
            self.pub1.publish(self.m)

        # Make brick magically float in the air.
        elif self.state == State.BRICK_FROZEN:

            # Broadcast world->brick transform and publish brick marker.
            
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

        # Make brick stationary if brick is on the floor.
        elif self.state == State.BRICK_FALLEN:

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
        """ Place brick at desired position.

            Args:
                request (float request.x, float request.y) : Time-stamped pose of target where the x and y positions correspond to the brick's location for catching, and the home position for returning.
                response : empty

            Returns:
                response : empty
        """

        self.get_logger().info("Placing Brick")

        # Current time at service call.
        time = self.get_clock().now().to_msg()

        # Initialize x and y position of brick.
        self.brick_x = request.x
        self.brick_y = request.y

        # Broadcast world->brick transform and publish brick marker.
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

        # Change state of brick to placed.
        self.state = State.BRICK_PLACED

        return response

    def drop_callback(self, request, response):
        """ Drop brick from placed position. Empty type service.
        """

        self.get_logger().info("Dropping Brick")

        self.state = State.BRICK_DROPPED

        return response



def arena(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
