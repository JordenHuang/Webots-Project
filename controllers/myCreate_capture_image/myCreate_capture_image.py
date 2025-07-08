from controller import Robot
from controller import Keyboard
import cv2
import numpy as np
import os

os.makedirs("images", exist_ok=True)

# Create the robot instance
myCreate = Robot()

# get the time step of the current world.
TIMESTEP = int(myCreate.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIMESTEP)

# ========== MyCreate ==========
left_motor = myCreate.getDevice("left wheel motor")
right_motor = myCreate.getDevice("right wheel motor")
left_motor.setPosition(float('+inf'));
right_motor.setPosition(float('+inf'));
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_position_sensor = myCreate.getDevice("left wheel sensor")
right_position_sensor = myCreate.getDevice("right wheel sensor")
left_position_sensor.enable(TIMESTEP)
right_position_sensor.enable(TIMESTEP)

camera_left = myCreate.getDevice("camera_left")
camera_right = myCreate.getDevice("camera_right")
camera_left.enable(TIMESTEP)
camera_right.enable(TIMESTEP)


counter = 0
x_pressed = False

width = camera_left.getWidth()
height = camera_left.getHeight()
if width != 1080 and height != 720:
    print("Check you camera width and height!")
    os.exit(1)

# Main loop
while myCreate.step(TIMESTEP) != -1:

    WHEEL_MAX_SPEED = 6.28
    FORWARD_RATIO = 1 # 0.75
    key = keyboard.getKey()
    if key == ord('W'):
        left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
    elif key == ord('S'):
        left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
    elif key == ord('A'):
        left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
    elif key == ord('D'):
        left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
    elif key == ord('X'):
        if not x_pressed:
            x_pressed = True
            camera_left.saveImage(f"images/cl_{counter:03}.png", None)
            camera_right.saveImage(f"images/cr_{counter:03}.png", None)
            print(f"Saved {counter:03}.png")
            counter += 1
    else:
        x_pressed = False
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)