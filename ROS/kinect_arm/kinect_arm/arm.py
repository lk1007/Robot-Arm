import struct
import serial
import time
import math


class angles_t:
    base: int
    shoulder: int
    elbow: int
    wrist: int
    hand: int

    def __init__(self, base: float, shoulder: float, elbow: float, wrist: float, hand: float):
        self.base = base
        self.shoulder = shoulder
        self.elbow = elbow
        self.wrist = wrist
        self.hand = hand

    def __getitem__(self, i):
        angles = [self.base, self.shoulder, self.elbow, self.wrist, self.hand]
        return angles[i]

    def __str__(self):
        return "base: " + str(self.base) + "\nshoulder: " + str(self.shoulder) + "\nelbow: " + str(self.elbow) + "\nwrist: " + str(self.wrist) + "\nhand: " + str(self.hand)


default_min_angles = angles_t(base=20, shoulder=40, elbow=0, wrist=60, hand=0)
default_max_angles = angles_t(base=170, shoulder=100, elbow=80, wrist=120, hand=50)

start_angles = angles_t(base=50, shoulder=70, elbow=50, wrist=50, hand=90)


class RobotArm:

    def angle_from_ros(self,ros_angle):
        return ros_angle*180/math.pi

    def __init__(self, serial_port, min_angles=default_min_angles, max_angles=default_max_angles):
        self.movable = True
        self.min_angles = min_angles
        self.max_angles = max_angles
        self.curr_angles = start_angles
        self.ser = serial_port
        self.send_angles()

    # Make sure angles are valid
    def check_angles(self, angles: angles_t):
        def check_angle(angle, min_angle, max_angle):
            if (angle < min_angle):
                angle = min_angle
            if (angle > max_angle):
                angle = max_angle
            return angle
        angles.base = check_angle(
            angles.base, self.min_angles.base, self.max_angles.base)
        angles.shoulder = check_angle(
            angles.shoulder, self.min_angles.shoulder, self.max_angles.shoulder)
        angles.elbow = check_angle(
            angles.elbow, self.min_angles.elbow, self.max_angles.elbow)
        angles.wrist = check_angle(
            angles.wrist, self.min_angles.wrist, self.max_angles.wrist)
        angles.hand = check_angle(
            angles.hand, self.min_angles.hand, self.max_angles.hand)
        return angles

    def send_angles(self, base=None, shoulder=None, elbow=None, wrist=None, hand=None):
        try:
            if (base != None):
                self.curr_angles.base = base
            if (shoulder != None):
                self.curr_angles.shoulder = shoulder
            if (elbow != None):
                self.curr_angles.elbow = elbow
            if (wrist != None):
                self.curr_angles.wrist = wrist
            if (hand != None):
                self.curr_angles.hand = hand
            #print(self.curr_angles)
            self.curr_angles = self.check_angles(self.curr_angles)
            angles = list(self.curr_angles)
            angles = [int(i) for i in angles]
            for angle in angles:
                if angle < 0:
                    raise ValueError("Negative numbers are not allowed")
            data = struct.pack('%sf' % len(angles), *angles)
            if (self.movable):
                self.ser.write(data)  # Assuming `ser` is defined elsewhere
        except Exception as e:
            print(e)

    def set_enable(self, enable : bool):
        self.movable = enable
    
    def set_angle_from_ros(self, name : str, angle : float):
        angle = self.angle_from_ros(angle)
        if(name == "world_to_base"):
            self.send_angles(base=angle)
        elif(name == "shoulder_joint"):
            self.send_angles(shoulder=angle)
        elif(name == "elbow_joint"):
            self.send_angles(elbow=angle)
        elif(name == "wrist"):
            self.send_angles(wrist=angle)

if __name__ == "__main__":
    arm = RobotArm(serial_port=serial.Serial(port='/dev/ttyACM0',baudrate=115200))
    def angle_test(): 
        arm.send_angles(base=80, shoulder=0, elbow=0, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=90, elbow=0, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=30, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=90, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=0, hand=0)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=0, hand=30)
        time.sleep(1)
        arm.send_angles(base=0, shoulder=0, elbow=0, wrist=0, hand=0)


    def example_2():
        arm.send_angles(base=80, shoulder=80, elbow=100, wrist=10, hand=10)
        time.sleep(1)
        arm.send_angles(hand=70)
        time.sleep(1)
        arm.send_angles(hand=10)
        time.sleep(1)
        arm.send_angles(hand=50)
        time.sleep(1)
        arm.send_angles(hand=90)
        time.sleep(1)


    arm.set_enable(True)
    angle_test()

