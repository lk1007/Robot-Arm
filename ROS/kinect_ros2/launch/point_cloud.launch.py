from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import math
pi_offset_y_in = 55
pi_offset_x_in = 4*12

def generate_launch_description():
    #proj_root_path = os.path.dirname(os.path.abspath(__file__)) + "/../"
    proj_root_path = get_package_share_directory('kinect_ros2') + '/'
    rviz_config_file = proj_root_path + 'config/thing.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file]
    )

    kinect_node = Node(
        package='kinect_ros2',
        executable='kinect_node',
        name='kinect_node',
        output='screen',
        parameters=[
            {'publish_pointcloud':True},
            {'publish_detection': True}
        ]
    )

    transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_to_kinect',
        arguments=[
            '0', '0', '0',        # x y z translation
            '0', '0', f'{-math.pi/2}',        # roll pitch yaw (in radians)
            'map',                # parent frame
            'kinect'   # child frame
        ],
        output='screen'
    )
    transform_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_to_pi',
        arguments=[
            f'{pi_offset_x_in*0.0254}', f'{pi_offset_y_in*.0254}', '0',        # x y z translation
            #f'0', '0', '0',        # roll pitch yaw (in radians)
            f'{math.pi/2}', f'{math.pi}', f'{-math.pi/2}',        # roll pitch yaw (in radians)
            'map',                # parent frame
            'pi'   # child frame
        ],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        kinect_node,
        transform_node,
        transform_node_2,
    ])