from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    proj_root_path = get_package_share_directory('arm_driver') + '/'
    config_path = proj_root_path + 'config'
    generate_joint_limits_yaml = ExecuteProcess(
        cmd=['python3', config_path + "/joint_limits.py"],
        output='screen'
    )
    generate_joint_zeros_yaml = ExecuteProcess(
        cmd=['python3', config_path + "/joint_zeros.py"],
        output='screen'
    )
    return LaunchDescription([
        generate_joint_limits_yaml,
        generate_joint_zeros_yaml,
    ])