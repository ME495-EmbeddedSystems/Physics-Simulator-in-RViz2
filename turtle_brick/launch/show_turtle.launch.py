# from ament_index_python.packages import get_packages_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution, TextSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp", default_value="gui",
                              description="choose to use joint_state_publisher or gui: gui, jsp, or none"),
        DeclareLaunchArgument(name="rviz_config", default_value=PathJoinSubstitution([FindPackageShare("turtle_brick"), "view_robot.rviz"]), 
                               description="Path to rviz config file"),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", LaunchConfiguration("rviz_config")]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description":
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                            [FindPackageShare("turtle_brick"), "turtle.urdf.xacro"])])}
            ]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "gui"))
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "jsp"))
        )
    ])