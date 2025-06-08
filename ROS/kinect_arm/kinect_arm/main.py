import rclpy
from ros_bridge import RobotArmRosListener
from arm import RobotArm
import serial

def main(args=None):
    rclpy.init(args=args)
    ser = serial.Serial(port='/dev/ttyACM0',baudrate=115200)
    arm = RobotArm(serial_port=ser)
    node = RobotArmRosListener(arm=arm)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()