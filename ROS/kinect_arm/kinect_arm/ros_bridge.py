#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from arm import RobotArm

class RobotArmRosListener(Node):
    def __init__(self, arm : RobotArm):
        super().__init__('joint_state_listener')
        self.arm = arm
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: JointState):
        self.get_logger().info("Received JointState:")
        for name, position in zip(msg.name, msg.position):
            self.arm.set_angle_from_ros(name = name, angle = position)
            #self.get_logger().info(f"  {name}: {position:.3f}")