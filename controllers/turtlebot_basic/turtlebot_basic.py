from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
from controller import Joystick
# import math
import time

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

# # Joystick
# joystick = Joystick()
# if not joystick.isConnected():
#     print("No joystick is connected")
#     exit(1)
# print(joystick.model)
# joystick.enable(TIMESTEP)

while turtlebot.step(TIMESTEP) != -1:
    key = None
    key = keyboard.getKey()

    if key == ord('W'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * -0.5)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * -0.5)
    elif key == ord('S'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * 0.5)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * 0.5)
    elif key == ord('A'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * -0.5)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * 0.5)
    elif key == ord('D'):
        wheel_left.setVelocity(WHEEL_MAX_SPEED * 0.5)
        wheel_right.setVelocity(WHEEL_MAX_SPEED * -0.5)
    else:
        wheel_left.setVelocity(0.0)
        wheel_right.setVelocity(0.0)