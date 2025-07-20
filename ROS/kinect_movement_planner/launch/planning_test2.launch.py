from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os,yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)

def generate_launch_description():
    package_name = 'kinect_movement_planner'
    proj_root_path = get_package_share_directory(package_name) + '/'
    arm_driver_root_path = get_package_share_directory('arm_driver') + '/'
     # Paths
    urdf_path = arm_driver_root_path + "urdf/arm.urdf.xacro"
    mesh_path = arm_driver_root_path + "meshes"
    config_path = proj_root_path + "/config"
    # Define xacro mappings for the robot description file

    generate_limits = ExecuteProcess(
        cmd=['python3', config_path + "/joint_limits.py"],
        output='screen'
    )

    urdf_launch_arguments = {
        "mesh_path": mesh_path
        #"robot_ip": "xxx.yyy.zzz.www",
        #"use_fake_hardware": "true",
        #"gripper": "robotiq_2f_85",
        #"dof": "7",
    }
    # Planning Functionality
    planning_pipelines_config = {
        "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1
        }
    }


    # Load the robot configuration
    moveit_config = MoveItConfigsBuilder(
            "kinect_movement_planner", package_name=package_name
        ).robot_description(file_path=urdf_path, mappings=urdf_launch_arguments) \
        .robot_description_semantic(file_path=f"{config_path}/kinect_arm.srdf") \
        .joint_limits(file_path=f"{config_path}/joint_limits.yaml") \
        .trajectory_execution(file_path=f"{config_path}/trajectory.yaml") \
        .planning_scene_monitor( 
            publish_robot_description=True, publish_robot_description_semantic=True
        ) \
        .planning_pipelines(
            default_planning_pipeline = "ompl",
            pipelines=["ompl"]
        ).to_moveit_configs()


    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    moveit_simple_controllers_yaml = load_yaml(
         f"{config_path}/trajectory.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[#moveit_config.to_dict(),
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    planning_pipelines_config,
                    moveit_config.joint_limits,
                    trajectory_execution,
                    {"controller_manager":"controller_manager/ControllerManager"},
                    moveit_controllers,
                    ]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", config_path + "/kinect_arm.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution
        ],
    )
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[f"{config_path}/kinect_arm_controllers.yaml"],
        remappings=[
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c","/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_controller", "-c", "/controller_manager"],
    )
    
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_gripper_controller", "-c", "/controller_manager"],
    )

    print("AAAAAAAAAAAAAAAAAAAA: ",moveit_config.to_dict()['planning_pipelines'])
    goalPublisher = Node(
        package=package_name,
        executable="goalPublisher",
        name="goalPublisher",
        output="screen",
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[
            planning_pipelines_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.trajectory_execution,
            moveit_config.joint_limits
            ]
    )

    return LaunchDescription(
            [
                generate_limits,
                rviz_node,
                static_tf,
                robot_state_publisher,
                run_move_group_node,
                ros2_control_node,
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                hand_controller_spawner,
                #goalPublisher
            ]
        )