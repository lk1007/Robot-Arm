from pynput.mouse import Listener, Button
import threading
import keyboard
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


min_angles = angles_t(base=20, shoulder=50, elbow=20, wrist=20, hand=0)
max_angles = angles_t(base=180, shoulder=120, elbow=180, wrist=180, hand=90)

start_angles = angles_t(base=50, shoulder=70, elbow=50, wrist=50, hand=90)


class RobotArm:

    def __init__(self, port, min_angles, max_angles):
        self.movable = False
        self.min_angles = min_angles
        self.max_angles = max_angles
        self.curr_angles = start_angles
        self.ser = serial.Serial(baudrate=115200, port=port)
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

    # send the angles over serial to robot stm32
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
            angles = [int(i) for i in angles]
            for angle in angles:
                if angle < 0:
                    raise ValueError("Negative numbers are not allowed")
            data = struct.pack('%sf' % len(angles), *angles)
            if (self.movable):
                self.ser.write(data)  # Assuming `ser` is defined elsewhere
        except Exception as e:
            print(e)


arm = RobotArm(port="COM5", min_angles=min_angles, max_angles=max_angles)

last = None


def on_move(x, y):
    global arm
    global last
    if (not last):
        last = x, y
    dx = (x - last[0])
    dy = (y - last[1])
    last = x, y
    target_base = arm.curr_angles.base - dx/3
    target_shoulder = arm.curr_angles.shoulder - dy/8
    if (target_base < arm.min_angles.base):
        target_base = arm.min_angles.base
    if (target_base > arm.max_angles.base):
        target_base = arm.max_angles.base
    if (target_shoulder < arm.min_angles.shoulder):
        target_shoulder = arm.min_angles.shoulder
    if (target_shoulder > arm.max_angles.shoulder):
        target_shoulder = arm.max_angles.shoulder
    # print(target_base, target_shoulder)
    arm.send_angles(base=target_base)
    arm.send_angles(shoulder=target_shoulder)

    # print('Mouse moved to ({0}, {1})'.format(x, y))


grab = -1


def on_click(x, y, button, pressed):
    global arm
    global grab
    if (button == Button.left and pressed):
        grab = grab * -1
        arm.send_angles(hand=50 + 40*grab)
    if pressed:
        print('Mouse clicked at ({0}, {1}) with {2} state {3}'.format(
            x, y, button, pressed))


start_wrist = None


def on_scroll(x, y, dx, dy):
    print(arm.curr_angles.elbow)
    arm.send_angles(elbow=arm.curr_angles.elbow + dy*5)
    # print('Mouse scrolled at ({0}, {1}) ({2}, {3})'.format(x, y, dx, dy))


def keyboard_thread_func():
    try:
        while (1):
            if (keyboard.is_pressed('z')):
                if (not arm.movable):
                    arm.movable = True
            if (keyboard.is_pressed('x')):
                if (arm.movable):
                    arm.movable = False
            if (keyboard.is_pressed('c')):
                arm.send_angles(wrist=arm.curr_angles.wrist + 5)
            if (keyboard.is_pressed('v')):
                arm.send_angles(wrist=arm.curr_angles.wrist - 5)
    except KeyboardInterrupt:
        return


keyboard_thread = threading.Thread(target=keyboard_thread_func)
# Collect events until released
with Listener(on_move=on_move, on_click=on_click, on_scroll=on_scroll) as listener:
    try:
        arm.movable = False
        keyboard_thread.start()

        keyboard_thread.join()
        listener.join()
    except KeyboardInterrupt:
        pass


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
    arm.send_angles(hand=70)
    time.sleep(1)
    arm.send_angles(hand=10)
    time.sleep(1)
    arm.send_angles(hand=50)
    time.sleep(1)
    arm.send_angles(hand=90)
    time.sleep(1)
