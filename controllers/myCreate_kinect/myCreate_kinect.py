from controller import Robot
from controller import Keyboard
from controller import Gyro
# import cv2
# import numpy as np
# import os
import math


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

inertial_unit = myCreate.getDevice("inertial unit")
inertial_unit.enable(TIMESTEP)

camera_width = camera.getWidth()
camera_height = camera.getHeight()

def getIMUDegrees(): #convert radians to degrees and make range 180
    return inertial_unit.getRollPitchYaw()[2] * 180/math.pi

def getDirectionFacing():
    degrees = (getIMUDegrees() + 360) % 360
    if(degrees < 22.5 or degrees > 337.5):
        return 'N'
    if(degrees < 337.5 and degrees > 292.5):
        return 'NE'
    if(degrees < 292.5 and degrees > 247.5):
        return 'E'
    if(degrees < 247.5 and degrees > 202.5):
        return 'SE'
    if(degrees < 202.5 and degrees > 157.5):
        return 'S'
    if(degrees < 157.5 and degrees > 112.5):
        return 'SW'
    if(degrees < 112.5 and degrees > 67.5):
        return 'W'
    if(degrees < 67.5 and degrees > 22.5):
        return 'NW'
    return '?'

def IMUPrint(): # print robot orientation in degrees
    print(' ')
    print('[IMU: '+ str(round(getIMUDegrees(), 1)) + '° ' + getDirectionFacing() + ']')
    # dist = getDistanceSensors()
    # print('[left, front, right] | ', end='')
    # print(dist)

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

    IMUPrint()