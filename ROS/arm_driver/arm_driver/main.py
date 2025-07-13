import rclpy
import arm_driver.ros_bridge as ros_bridge
from arm_driver.arm import RobotArm
import serial

def main(args=None):
    rclpy.init(args=args)
    node = ros_bridge.RobotArmRosListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()