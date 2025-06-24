import rclpy
import arm_driver.ros_bridge as ros_bridge
from arm_driver.arm import RobotArm
import serial

def main(args=None):
    rclpy.init(args=args)
    ser = serial.Serial(port='/dev/ttyACM0',baudrate=115200)
    arm = RobotArm(serial_port=ser)
    node = ros_bridge.RobotArmRosListener(arm=arm)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()