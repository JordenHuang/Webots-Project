from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
from controller import Joystick
# import math
import time

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

# Main loop
while turtlebot.step(TIMESTEP) != -1:
    key = None
    key = keyboard.getKey()

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
        # print(f"button: {joystick.getPressedButton()}")
        # joystick_btn = joystick.getPressedButton()
        # if joystick_btn == 4:
        # elif joystick_btn == 5:

