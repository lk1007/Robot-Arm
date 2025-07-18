from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    proj_root_path = get_package_share_directory('arm_driver') + '/'
    arm_driver = Node(
        package="arm_driver",
        executable="arm_node",
        name="arm_driver",
        output="screen",
        parameters=[
            {'test_joint_listener':True},
        ]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([proj_root_path + "launch", 'render_arm.launch.py'])
        ),
        arm_driver,
    ])