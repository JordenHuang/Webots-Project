from controller import Robot
from controller import wb
from controller import PositionSensor
from controller import LED
from controller import Receiver
from controller import TouchSensor
from controller import Motor
from controller import DistanceSensor
from controller import Keyboard
from controller import Joystick
import sys
import time
import math
import random

PI = math.pi

MAX_SPEED  = 16
NULL_SPEED = 0
HALF_SPEED = 8
MIN_SPEED  = -16

WHEEL_RADIUS       = 0.031
AXLE_LENGTH        = 0.271756
ENCODER_RESOLUTION = 507.9188

# Create the robot instance
myCreate = Robot()

# get the time step of the current world.
TIMESTEP = int(myCreate.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIMESTEP)

bumpers = {
    "bumper_left" : None,
    "bumper_right": None,
}

cliffSensors = {
    "cliff_left"       : None,
    "cliff_front_left" : None,
    "cliff_front_right": None,
    "cliff_right"      : None,
}

leds = {
    "led_on"  : None,
    "led_play": None,
    "led_step": None
}

receivers = {
    "receiver": None
}

left_motor            = None
right_motor           = None
left_position_sensor  = None
right_position_sensor = None

# Cameras
camera_left = None
camera_right = None

def step():
    if myCreate.step(TIMESTEP) == -1:
        sys.exit(0)

def initDevices():
    global left_motor, right_motor, left_position_sensor, right_position_sensor, camera_left, camera_right
    for name in bumpers.keys():
        bumpers[name] = myCreate.getDevice(name)
        bumpers[name].enable(TIMESTEP)

    for name in cliffSensors.keys():
        cliffSensors[name] = myCreate.getDevice(name)
        cliffSensors[name].enable(TIMESTEP)

    for name in leds.keys():
        leds[name] = myCreate.getDevice(name)

    for name in receivers.keys():
        receivers[name] = myCreate.getDevice(name)
        receivers[name].enable(TIMESTEP)

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

def is_there_a_virtual_wall() -> bool:
    return receivers["receiver"].getQueueLength() > 0

def is_there_a_collision_at_left() -> bool:
    return bumpers["bumper_left"].getValue() != 0.0

def is_there_a_collision_at_right() -> bool:
    return bumpers["bumper_right"].getValue() != 0.0

def fflush_ir_receiver() -> bool:
    while receivers["receiver"].getQueueLength() > 0:
        receivers["receiver"].getBytes();

def is_there_a_virtual_wall() -> bool:
    return receivers["receiver"].getQueueLength() > 0

def is_there_a_cliff_at_left() -> bool:
    return cliffSensors["cliff_left"].getValue() < 100.0 or cliffSensors["cliff_front_left"].getValue() < 100.0

def is_there_a_cliff_at_right() -> bool:
    return cliffSensors["cliff_right"].getValue() < 100.0 or cliffSensors["cliff_front_right"].getValue() < 100.0

def is_there_a_cliff_at_front() -> bool:
    return cliffSensors["cliff_front_left"].getValue() < 100.0 or cliffSensors["cliff_front_right"].getValue() < 100.0

def goForward():
    left_motor.setVelocity(MAX_SPEED);
    right_motor.setVelocity(MAX_SPEED);

def goBackward():
    left_motor.setVelocity(-HALF_SPEED);
    right_motor.setVelocity(-HALF_SPEED);

def stop():
    left_motor.setVelocity(-NULL_SPEED);
    right_motor.setVelocity(-NULL_SPEED);

def passiveWait(sec):
    start_time = myCreate.getTime()
    step()
    while start_time + sec > myCreate.getTime():
        step()

def turn(angle: float):
    stop()
    l_offset = left_position_sensor.getValue()
    r_offset = right_position_sensor.getValue()
    step()
    neg = -1.0 if angle < 0.0 else 1.0
    left_motor.setVelocity(neg * HALF_SPEED)
    right_motor.setVelocity(-neg * HALF_SPEED)
    while True:
        l = left_position_sensor.getValue() - l_offset
        r = right_position_sensor.getValue() - r_offset
        dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
        dr = r * WHEEL_RADIUS                        # distance covered by right wheel in meter
        orientation = neg * (dl - dr) / AXLE_LENGTH  # delta orientation in radian
        step()
        if not (orientation < neg * angle):
            break
    stop()
    step()

def mainLoop():
    leds["led_on"].set(True)
    passiveWait(0.5);
    while True:
        if is_there_a_virtual_wall():
            print("Virtual wall detected")
            turn(PI)
        elif is_there_a_collision_at_left() or is_there_a_cliff_at_left():
            print("Left obstacle detected")
            goBackward()
            passiveWait(0.5)
            turn(PI * random.random())
        elif is_there_a_collision_at_right() or is_there_a_cliff_at_right() or is_there_a_cliff_at_front():
            print("Right obstacle detected")
            goBackward()
            passiveWait(0.5)
            turn(-PI * random.random())
        else:
            goForward()
        
        fflush_ir_receiver()
        step()

        # key = None
        # key = keyboard.getKey()

        # if key == ord('W'):
        # elif key == ord('S'):
        # elif key == ord('A'):
        # elif key == ord('D'):
        # else:

if __name__ == "__main__":
    initDevices()
    print("[INFO] myCreate all devices initialized successfully!")

    mainLoop()