#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from kinect_arm.arm import RobotArm
from control_msgs.msg import JointTrajectoryControllerState


class RobotArmRosListener(Node):
    def __init__(self, arm : RobotArm):
        super().__init__('joint_state_listener')
        self.declare_parameter('test_joint_listener', False)
        self.arm = arm
        if(self.get_parameter('test_joint_listener').get_parameter_value().bool_value):
            self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.listener_callback,
                10)
        else:
            self.subscription = self.create_subscription(
                JointTrajectoryControllerState,
                '/robot_arm_controller/controller_state',
                self.trajectory_callback,
                10
            )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self.arm.set_angle_from_ros(name = name, angle = position)

    def trajectory_callback(self, msg: JointTrajectoryControllerState):
        print("SSSSs:",msg.feedback.positions)
        #self.get_logger().info(f"Received trajectory with {len(msg.points)} points")

        #for idx, point in enumerate(msg.points):
            #positions = point.positions
            #time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            #self.get_logger().info(
                #f"Point {idx}: positions={positions} time_from_start={time_from_start:.2f}s"
            #)
            ## TODO: Send this to your actual hardware using your driver interface

