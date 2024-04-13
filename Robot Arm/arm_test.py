import struct
import serial
import time


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


min_angles = angles_t(base=20, shoulder=20, elbow=20, wrist=20, hand=20)
max_angles = angles_t(base=100, shoulder=100, elbow=100, wrist=100, hand=100)


class RobotArm:

    def __init__(self, port, min_angles, max_angles):
        self.min_angles = min_angles
        self.max_angles = max_angles
        self.curr_angles = min_angles
        self.ser = serial.Serial(baudrate=115200, port=port)

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
            if (base):
                self.curr_angles.base = base
            if (shoulder):
                self.curr_angles.shoulder = shoulder
            if (elbow):
                self.curr_angles.elbow = elbow
            if (wrist):
                self.curr_angles.wrist = wrist
            if (hand):
                self.curr_angles.hand = hand
            self.curr_angles = self.check_angles(self.curr_angles)
            angles = list(self.curr_angles)
            for angle in angles:
                if angle < 0:
                    raise ValueError("Negative numbers are not allowed")
            data = struct.pack('%sf' % len(angles), *angles)
            self.ser.write(data)  # Assuming `ser` is defined elsewhere
        except Exception as e:
            print(e)


arm = RobotArm(port="COM15", min_angles=min_angles, max_angles=max_angles)


def example_1():
    arm.send_angles(base=80, shoulder=80, elbow=100, wrist=10, hand=10)
    time.sleep(1)
    arm.send_angles(base=60, shoulder=60, elbow=70, wrist=50, hand=10)
    time.sleep(1)
    arm.send_angles(base=80, shoulder=80, elbow=100, wrist=10, hand=10)
    time.sleep(1)


def example_2():
    arm.send_angles(base=80, shoulder=80, elbow=100, wrist=10, hand=10)
    time.sleep(1)
    arm.send_angles(elbow=70)
    time.sleep(1)
    arm.send_angles(elbow=10)
    time.sleep(1)
    arm.send_angles(elbow=50)
    time.sleep(1)
    arm.send_angles(elbow=90)
    time.sleep(1)

example_2()
