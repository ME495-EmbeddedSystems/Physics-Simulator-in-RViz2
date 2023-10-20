"""A node that drives the turtle robot to goalposes and back, after which the turtle dumps the brick. """

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Vector3, PoseStamped
from tf2_ros.transform_listener import TransformListener
from math import pi
from .quaternion import angle_axis_to_quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from turtle_brick_interfaces.msg import Tilt
from enum import Enum, auto
import math
from rcl_interfaces.msg import ParameterDescriptor


class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration.
    """
    WAITING = auto(),
    CATCHING = auto(),
    FOCUSING = auto(),
    RETURNING = auto(),
    DROPPING = auto()

def turtle_twist(xdot, ydot, omega):
    """ Create a twist suitable for a turtle.

        Args:
           xdot (float) : the forward component of linear velocity
           ydot (float) : the lateral component of linear velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = xdot, y = ydot, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = omega))

class RunTurtle(Node):
    """
    Controls the motion of the turtle.

    Static Broadcasts:
       world -> odom
    Broadcasts:
       odom -> base_link
    Joint States:
        wheel_axle
        swivel
        tip

    PUBLISHES:
        /joint_states : Joint states of the turtle robot
        /odom : Time-stamped pose and twist 
        cmd_vel : Twist of turtlesim

    SUBSCRIBES:
        /goal_pose : 
        turtle1/pose :
        tilt :

    """

    def __init__(self):
        super().__init__('run_turtle')
    
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
                               ParameterDescriptor(description="Downwards acceleration due to gravity"))
        self.gravity = self.get_parameter("gravity_accel").get_parameter_value().double_value

        # Static broadcasters publish on /tf_static.
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Position of odom frame in world frame. Orientation is identical.
        self.homeX = 5.5 # X-ofsett mimicking turtlesim
        self.homeY = 5.5 # Y-offset mimicking turtlesim
        self.homeZ = 0.957 # Z-offset is base_link height above the ground

        # Create the static transform world -> odom.
        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"
        world_odom_tf.transform.translation.x = float(self.homeX)
        world_odom_tf.transform.translation.y = float(self.homeY)
        world_odom_tf.transform.translation.z = float(self.homeZ)
        self.static_broadcaster.sendTransform(world_odom_tf) # Broadcast world->odom transform
        self.get_logger().info("Static Transform: world->odom")

        # Robot initial configuration.
        self.wheel_pos = 0 # Angle of wheel axle
        self.tip_pos = 0 # Tilted angle of platform
        self.x = 0 # X-position in odom frame
        self.y = 0 # Y-position in odom frame
        self.z = 0 # Z-position in odom frame
        self.theta = 0 # Angle about +ve Z-axis in odom frame

        # Create transform odom -> base_link.
        self.broadcaster = TransformBroadcaster(self)

        # Initialize additional variables.

        # Target position for turtle robot in world frame.
        self.world_targetX = self.homeX
        self.world_targetY = self.homeY

        # Target position for turtle robot in odom frame.
        self.targetX = self.world_targetX - self.homeX
        self.targetY = self.world_targetY - self.homeY

        # Initial State.
        self.state = State.WAITING 
        
        # Distance tolerance for reaching goal.
        self.dtol = 0.05

        # Velocities.
        self.wheel_omg = self.speed / self.wheel_radius  # Angular velocity of wheels
        self.swivel_omg = 0.0  # Angular velocity of heading of the robot
        self.vx = self.speed * math.cos(self.theta) # X-component of linear velocity
        self.vy = self.speed * math.sin(self.theta) # Y-component of linear velocity

        # Create a timer to do the rest of the transforms.
        self.frequency = 100.0 # Node frequency
        self.dt = 1/self.frequency # Node timestep
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # Topics this node publishes to.

        # Create publisher for Joint States.
        self.positioncommander = self.create_publisher(JointState, 'joint_states', 10)

        # Create publisher for Odometery messages.
        self.odometry = self.create_publisher(Odometry, 'odom', 10)

        # Create Publisher for /cmd_vel topic.
        self.cmdvel = self.create_publisher(Twist, "cmd_vel", 10)

        # Topics this node subscribes to.

        # Create goalpose subscriber.
        self.goalpose_subscriber = self.create_subscription(PoseStamped, "goal_pose", self.update_goalpose, 10)
        self.goalpose = PoseStamped()

        # Create turtle pose subscriber.
        self.pose_subscriber = self.create_subscription(Pose, "turtle1/pose", self.update_turtlepose, 10)
        self.pose = Pose()

        # Create tilt subscriber.
        self.tilt_subscriber = self.create_subscription(Tilt, "tilt", self.update_tilt, 10)

    def timer_callback(self):
        """ Timer Callback for this node. Dictates motion of turtle based on its state, goal positon, and tilt angle.
        """

        # Current time for every iteration.
        time = self.get_clock().now().to_msg()

        # Wait for brick to fall and new goalpose to be issued.
        if self.state == State.WAITING:

            self.vx = 0.0
            self.vy = 0.0
            self.wheel_omg = 0.0

        # Travel to goal position to catch the brick.
        elif self.state == State.CATCHING:

            # Stop after arriving
            if math.dist([self.x,self.y],[self.targetX,self.targetY]) <= self.dtol:

                self.state = State.FOCUSING
                self.get_logger().info("Waiting for brick to reach")
            

        # Stand steady until brick is caught.
        elif self.state == State.FOCUSING:

            self.vx = 0.0
            self.vy = 0.0
            self.wheel_omg = 0.0

        # Return to home position after brick is caught.
        elif self.state == State.RETURNING:

            # Stop after reaching home
            if math.dist([self.x,self.y],[0,0]) <= self.dtol:

                self.state = State.DROPPING
                self.get_logger().info("Reached home")

            # Keep updating target as home position.
            else:

                self.world_targetX = self.homeX
                self.world_targetY = self.homeY

                self.targetX = self.world_targetX - self.homeX
                self.targetY = self.world_targetY - self.homeY
                
        # Tilt the robot's platform after reaching home.
        elif self.state == State.DROPPING:

            self.state = State.WAITING

        # Update and publish joint states.

        # Calculate position of robot.
        self.x = self.x + self.vx * self.dt
        self.y = self.y + self.vy * self.dt

        # Calculate wheel joint angle.
        self.wheel_pos = float(self.wheel_pos + self.wheel_omg * self.dt)

        # Update joint state messages
        joints_states_msg = JointState()
        joints_states_msg.header.stamp = time
        joints_states_msg.name = ['wheel_axle','swivel','tip']
        joints_states_msg.position = [self.wheel_pos, self.theta, self.tip_pos]

        # Publish joint state messages
        self.positioncommander.publish(joints_states_msg)

        # Update odometry frame.
        
        # Update odom->base_link transform.
        odom_base = TransformStamped()
        odom_base.header.stamp = time
        odom_base.header.frame_id = "odom"
        odom_base.child_frame_id = "base_link"
        odom_base.transform.translation.x = float(self.x)
        odom_base.transform.translation.y = float(self.y)
        odom_base.transform.rotation = angle_axis_to_quaternion(0, [0, 0, 1])

        # Broadcast odom->base_link transform.
        self.broadcaster.sendTransform(odom_base)

        # Format odometry message.
        odometry_msg = Odometry()
        odometry_msg.header.stamp = time
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.pose.pose.position.x = float(self.x)
        odometry_msg.pose.pose.position.y = float(self.y)
        odometry_msg.pose.pose.position.z = float(self.z)
        odometry_msg.pose.pose.orientation = angle_axis_to_quaternion(0, [0, 0, 1])
        odometry_msg.twist.twist.linear.x = self.vx
        odometry_msg.twist.twist.linear.y = self.vy
        odometry_msg.twist.twist.angular.z = self.swivel_omg

        # Publish odometry message.
        self.odometry.publish(odometry_msg)

        # Control motion of turtlesim
        
        # Publish cmd_vel message
        cmdvel_msg = turtle_twist(self.vx, self.vy, self.swivel_omg)
        self.cmdvel.publish(cmdvel_msg)
        
        # Update velocities

        self.theta = math.atan2(self.targetY - self.y, self.targetX - self.x)
        self.vx = self.speed * math.cos(self.theta)
        self.vy = self.speed * math.sin(self.theta)
        self.wheel_omg = self.speed / self.wheel_radius

    def update_goalpose(self, data):
        """ Update target position from /goal_pose topic.

            Args:
                data (PoseStamped) : Time-stamped pose of target where the x and y positions correspond to the brick's location for catching, and the home position for returning.
        """

        self.goalpose = data

        # Update target.

        self.world_targetX = self.goalpose.pose.position.x 
        self.world_targetY = self.goalpose.pose.position.y

        self.targetX = self.world_targetX - self.homeX
        self.targetY = self.world_targetY - self.homeY

        # Change state to go to the brick when it is dropped.
        if self.state == State.WAITING:

            self.state = State.CATCHING
            self.get_logger().info("Going to goal")

        # Change state to return home after catching brick or if brick is unreachable.
        elif self.state == State.FOCUSING:

            self.state = State.RETURNING
            self.get_logger().info("Returning home")

    def update_turtlepose(self, data):
        """ Update turtlesim position from turtle1/pose topic.

            Args:
                data (Pose) : pose of turtlesim.
        """

        self.turtlepose = data

    def update_tilt(self, data):
        """ Update incline of the platform of the robot.

            Args:
                data (Tilt) : tilt_angle field contains the angle of the platform with the horizontal.
        """

        self.tip_pos = data.tilt_angle

def main(args=None):
    rclpy.init(args=args)
    node = RunTurtle()
    rclpy.spin(node)
    rclpy.shutdown()
