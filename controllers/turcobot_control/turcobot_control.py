from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
from controller import Joystick
# import math
import time

import cv2
idx = 0

''' Joint angle constrants
J1: -168 ~ 168 (2.9321506666666663)
J2: -135 ~ 135 (2.3561924999999997)
J3: -150 ~ 150 (2.6179916666666667)
J4: -145 ~ 145 (2.530725277777778)
J5: -165 ~ 165 (2.8797908333333333)
J6: -180 ~ 180 (3.14159)
'''

PI = 3.14159
FORWARD_RATIO = 0.5
TURN_RATIO = 0.85

JOYSTICK_ENABLED = False

# create the Robot instance.
turtlebot = Robot()

# get the time step of the current world.
TIMESTEP = int(turtlebot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(TIMESTEP)

# Cameras
camera_left = turtlebot.getDevice("camera_left")
camera_right = turtlebot.getDevice("camera_right")
camera_left.enable(TIMESTEP)
camera_right.enable(TIMESTEP)

# Wheels
wheel_left = turtlebot.getDevice("left_wheel_motor")
wheel_right = turtlebot.getDevice("right_wheel_motor")
wheel_left.setPosition(float('+inf'));
wheel_right.setPosition(float('+inf'));
wheel_left.setVelocity(0.0)
wheel_right.setVelocity(0.0)
WHEEL_MAX_SPEED = 6.28


# Joystick
joystick = turtlebot.getJoystick()
if joystick is not None:
    joystick.enable(TIMESTEP)
    # Joystick will be active only after the first sampling period elapsed
    turtlebot.step(TIMESTEP)

if not joystick.isConnected():
    print("No joystick is connected")
    print("Use keyboard to control")
else:
    JOYSTICK_ENABLED = True
    print("Joystick connected")
    print(f"Joystick model: {joystick.model}")


joints = []
for i in range(6):
    joints.append(turtlebot.getDevice(f"joint{i}_rotational_motor"))

gripper_right_base_outer = turtlebot.getDevice("gripper_right::base_outer::rotational_motor")
gripper_right_outer_paddle = turtlebot.getDevice( "gripper_right::outer_paddle::rotational_motor")
gripper_right_inner = turtlebot.getDevice("gripper_right::base_inner::rotational_motor")

gripper_left_base_outer = turtlebot.getDevice("gripper_left::base_outer::rotational_motor")
gripper_left_outer_paddle = turtlebot.getDevice( "gripper_left::outer_paddle::rotational_motor")
gripper_left_inner = turtlebot.getDevice("gripper_left::base_inner::rotational_motor")

def mycobot_send_angles(degrees: list, speed=0.05):
    rad = [d * (PI / 180) for d in degrees]  # Convert degrees to radians
    current_angles = [j.getTargetPosition() for j in joints]  # Get current joint angles

    while any(abs(curr - target) > 0.01 for curr, target in zip(current_angles, rad)):  # Loop until close enough
        for i in range(len(joints)):
            diff = rad[i] - current_angles[i]
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angles[i] += step * (1 if diff > 0 else -1)  # Move in the correct direction
            joints[i].setPosition(current_angles[i])  # Apply new position
        
        # time.sleep(0.01)  # Control loop timing
        turtlebot.step(TIMESTEP)  # Small delay to control speed

def mycobot_gripper_send_angle(degree: int, speed=0.007):
    rad_right = degree * (PI / 180)  # Convert degrees to radians
    current_angle_right = gripper_right_base_outer.getTargetPosition()  # Get current joint angles

    rad_left = (-degree) * (PI / 180)  # Convert degrees to radians
    current_angle_left = gripper_left_base_outer.getTargetPosition()  # Get current joint angles

    flag = False
    while True:  # Loop until close enough
        flag = False
        # Right
        if abs(current_angle_right - rad_right) > 0.01:
            diff = rad_right - current_angle_right
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angle_right += step * (1 if diff > 0 else -1)  # Move in the correct direction

            gripper_right_base_outer.setPosition(current_angle_right)  # Apply new position
            gripper_right_outer_paddle.setPosition(-current_angle_right)  # Apply new position
            gripper_right_inner.setPosition(current_angle_right)  # Apply new position
        else:
            flag = True

        # Left
        if abs(current_angle_left - rad_left) > 0.01:
            diff = rad_left - current_angle_left
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angle_left += step * (1 if diff > 0 else -1)  # Move in the correct direction

            gripper_left_base_outer.setPosition(current_angle_left)  # Apply new position
            gripper_left_outer_paddle.setPosition(-current_angle_left)  # Apply new position
            gripper_left_inner.setPosition(current_angle_left)  # Apply new position
        else:
            if flag == True:
                break
        
        turtlebot.step(2*TIMESTEP)  # Small delay to control speed


# Initial position
degrees = [0, -135, 150, -120, 90, 0]
mycobot_send_angles(degrees)

mode = 0

# Main loop
while turtlebot.step(TIMESTEP) != -1:
    key = None
    key = keyboard.getKey()

    # TurtleBot movement
    if key == ord('W'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
    elif key == ord('S'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
    elif key == ord('A'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
    elif key == ord('D'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
    else:
        wheel_left.setVelocity(0.0)
        wheel_right.setVelocity(0.0)

    if key == ord('X'):
        camera_left.saveImage(f"cl_{idx}.png", None)
        camera_right.saveImage(f"cr_{idx}.png", None)
        print("Image saved")
        idx += 1

    # myCobot and gripper
    if key == ord('0'):
        print('Change to mode 0')
        mode = 0
    elif key == ord('1'):
        print('Change to mode 1')
        mode = 1
    elif key == ord('2'):
        print('Change to mode 2')
        degrees = [0, -135, 150, -125, 90, 0]
        mycobot_send_angles(degrees)
        mode = 2
    elif key == ord('3'):
        print('Change to mode 3')
        mode = 3
    elif key == ord('4'):
        print('Change to mode 4')
        mode = 4
    elif key == ord('5'):
        print('Change to mode 5')
        mode = 5
    elif key == ord('6'):
        print('Change to mode 6')
        mode = 6

    # myCobot
    if mode == 1:
        degrees = [0, 0, 0, 0, 165, 0]
        mycobot_send_angles(degrees)
        mode = 0
    elif mode == 2:
        degrees = [0, -135, 150, -125, 90, 0]
        mycobot_send_angles(degrees)
        mode = 0
    elif mode == 3:
        degrees = [0, 110, 5, -15, 165, 0]
        mycobot_send_angles(degrees, 0.02)
        mode = 0

    # Gripper
    elif mode == 4:
        deg = 30
        mycobot_gripper_send_angle(deg)
        mode = 0
    elif mode == 5:
        deg = -30
        mycobot_gripper_send_angle(deg)
        degrees = [0, 127, -5, -25, 165, 0]
        mode = 0
    
    elif mode == 6:
        degrees = [0, 80, 15, -35, 165, 0]
        mycobot_send_angles(degrees, 0.02)
        mode = 0



    if JOYSTICK_ENABLED:
        forward = joystick.getAxisValue(1)  # Y-axis for forward/backward
        turn = joystick.getAxisValue(0)  # X-axis for turning
        # print(f"forward: {forward}")
        # print(f"turn: {turn}")

        if forward != -1 and forward != 0:
            # Forward
            if forward < 0:
                # Forward right
                if turn > 0:
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -TURN_RATIO)
                # Forward left
                elif turn < 0:
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * -TURN_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                # Forward only
                else:
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            # Backward
            else:
                # Backward right
                if turn > 0:
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * TURN_RATIO)
                # Backward left
                elif turn < 0:
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * TURN_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                # Backward only
                else:
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif turn != 0:
            # Trun right
            if turn > 0:
                wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            # Trun left
            elif turn < 0:
                wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        else:
            wheel_left.setVelocity(0.0)
            wheel_right.setVelocity(0.0)

        # NOTE: Each physical joystick can be used by one controller at a time only
        joystick_btn = joystick.getPressedButton()
        if joystick_btn!= -1: print(f"button: {joystick_btn}")
        if joystick_btn == 0:
            degrees = [0, 80, 15, -35, 165, 0]
            mycobot_send_angles(degrees, 0.02)
        elif joystick_btn == 1:
            degrees = [0, 0, 0, 0, 165, 0]
            mycobot_send_angles(degrees)
        elif joystick_btn == 2:
            degrees = [0, -135, 150, -125, 90, 0]
            mycobot_send_angles(degrees)
        elif joystick_btn == 3:
            degrees = [0, 110, 5, -15, 165, 0]
            mycobot_send_angles(degrees, 0.02)

        # Gripper
        elif joystick_btn == 4:
            deg = 30
            mycobot_gripper_send_angle(deg)
        elif joystick_btn == 5:
            deg = -5
            mycobot_gripper_send_angle(deg)

