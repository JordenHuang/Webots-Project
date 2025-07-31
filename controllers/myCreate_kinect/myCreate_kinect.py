from controller import Robot
from controller import Keyboard
import cv2
import numpy as np
import os


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

camera = myCreate.getDevice("kinect color")
camera_range = myCreate.getDevice("kinect range")
camera.enable(TIMESTEP)
camera_range.enable(TIMESTEP)

camera_width = camera.getWidth()
camera_height = camera.getHeight()

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
    else:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)