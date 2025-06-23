from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #proj_root_path = os.path.dirname(os.path.abspath(__file__)) + "/../"
    proj_root_path = get_package_share_directory('kinect_arm') + '/'
    default_mesh_path = proj_root_path + "meshes"
    rviz_config_file = proj_root_path + 'config/thing.rviz'
    xacro_path = proj_root_path + 'urdf/arm.urdf.xacro'



    urdf_path = ParameterValue(
        Command(['xacro', ' ', xacro_path, ' mesh_path:=' + default_mesh_path]),
        value_type=str
    )

    arm_driver = Node(
        package="kinect_arm",
        executable="arm_node",
        name="arm_driver",
        output="screen",
        parameters=[
            {'test_joint_listener':True},
        ]
    )
    joint_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        output="screen"
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': urdf_path}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        arm_driver,
        joint_publisher
    ])