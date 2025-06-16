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

import cv2
import numpy as np

from direction import Direction
import robot_util as ru
from my_model import *
from abstacle_avoidance_algorithm import *

mannual_control = False
flag = False
fl = True

last_direction = None
oscillate_count = 0

stuck_count = 0

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


def obstacle_avoidance_main_loop():
    global left_motor, right_motor, left_position_sensor, right_position_sensor, camera_left, camera_right
    global mannual_control, flag, fl, last_direction, oscillate_count, stuck_count

    while myCreate.step(TIMESTEP) != -1:
        key = keyboard.getKey()

        if key == ord('M'):
            if not flag:
                if not mannual_control: print("Switch mode --> Mannual")
                else: print("Switch mode --> Obstacle avoidence")
                mannual_control = not mannual_control
                flag = True
        else:
            flag = False

        raw = camera_left.getImage()  # returns a byte string
        raw2 = camera_right.getImage()  # returns a byte string

        width = camera_left.getWidth()
        height = camera_left.getHeight()

        # Convert to a NumPy array, reshape, and convert RGB to BGR
        image_np = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))  # RGBA
        camImage = image_np[:, :, :3]  # Drop alpha channel
        camImage = cv2.cvtColor(camImage, cv2.COLOR_BGR2RGB)
        image_np2 = np.frombuffer(raw2, dtype=np.uint8).reshape((height, width, 4))  # RGBA
        camImage2 = image_np2[:, :, :3]  # Drop alpha channel
        camImage2 = cv2.cvtColor(camImage2, cv2.COLOR_BGR2RGB)

        if not mannual_control:
            # Semantic segmentation
            image = predict_image(None, camImage, False)
            image2 = predict_image(None, camImage2, False)

            # Find controlling point
            middleLineImage = np.copy(image)
            middleLineImage = cv2.cvtColor(middleLineImage, cv2.COLOR_GRAY2BGR)

            # Some parameters
            # row_line_ok = 155 # 128 + 32
            row_line_ok = 180 # 128 + 32
            row_line_back = 200 # 230
            range_accept = 10 #80
            image_middle_col = int(image.shape[1]/2)

            # Find the controlling point
            cpDot, middleDots = getControlPoint(image, row_line_ok)
            cpDot2, middleDots2 = getControlPoint(image2, row_line_ok)
            if cpDot:
                print(f"cp dot: {cpDot}")
                if cpDot2:
                    if abs(cpDot[0] - cpDot2[0]) < 30: # 消除地板分割出現的小黑點(錯誤的地板判斷)
                        cpDot = max(cpDot, cpDot2, key=lambda x: x[0])
                    # cpDot = [(cpDot[0] + cpDot2[0]) // 2, ((cpDot[1][0]+cpDot2[1][0])//2, (cpDot[1][1]+cpDot2[1][1])//2)]
                print(f"cp dot after avg: {cpDot}")

                for dot in middleDots:
                    cv2.circle(middleLineImage, dot, 2, (0, 255, 0), cv2.FILLED)
                cv2.circle(middleLineImage, cpDot[1], 3, (255, 0, 255), cv2.FILLED)

                # Draw the row line
                cv2.line(middleLineImage, (0, row_line_ok), (middleLineImage.shape[1], row_line_ok), (0, 0, 255), 1)
                cv2.line(middleLineImage, (0, row_line_back), (middleLineImage.shape[1], row_line_back), (0, 0, 127), 1)
                cv2.line(middleLineImage, (image_middle_col-range_accept, 0), (image_middle_col-range_accept, middleLineImage.shape[0]), (0, 0, 127), 1)
                cv2.line(middleLineImage, (image_middle_col+range_accept, 0), (image_middle_col+range_accept, middleLineImage.shape[0]), (0, 0, 127), 1)
            else:
                print("cpDot is none")
            cv2.imshow("camera right", cv2.cvtColor(camImage, cv2.COLOR_RGB2BGR))
            cv2.imshow("result", cv2.cvtColor(middleLineImage, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

        if mannual_control == True:
            # TurtleBot movement
            if key == ord('W'):
                ru.wheel_turn(left_motor, Direction.FORWARD, 1)
                ru.wheel_turn(right_motor, Direction.FORWARD, 1)
            elif key == ord('S'):
                ru.wheel_turn(left_motor, Direction.BACKWARD, 1)
                ru.wheel_turn(right_motor, Direction.BACKWARD, 1)
            elif key == ord('A'):
                ru.wheel_turn(left_motor, Direction.BACKWARD, 1)
                ru.wheel_turn(right_motor, Direction.FORWARD, 1)
            elif key == ord('D'):
                ru.wheel_turn(left_motor, Direction.FORWARD, 1)
                ru.wheel_turn(right_motor, Direction.BACKWARD, 1)
            else:
                ru.wheel_turn(left_motor, Direction.STOP, 0)
                ru.wheel_turn(right_motor, Direction.STOP, 0)
            # if fl:
            #     fl = False
            #     ru.wheel_turn(left_motor, Direction.FORWARD, 0.5)
            #     ru.wheel_turn(right_motor, Direction.BACKWARD, 0.5)
            #     myCreate.step(TIMESTEP*72)
            #     myCreate.step(TIMESTEP*72)
            #     myCreate.step(TIMESTEP*72)
            #     myCreate.step(TIMESTEP*72)
            # else:
            #     ru.wheel_turn(left_motor, Direction.STOP, 0)
            #     ru.wheel_turn(right_motor, Direction.STOP, 0)

        else:
            if cpDot:
                # Stuck detection
                if stuck_count > 6:
                    stuck_count = 0
                    print("To back because it stucks")
                    ru.wheel_turn(left_motor, Direction.BACKWARD, 0.55)
                    ru.wheel_turn(right_motor, Direction.BACKWARD, 0.55)
                    for _ in range(3):
                        myCreate.step(TIMESTEP * 20)
                # Oscillate detection
                elif oscillate_count > 4:
                    oscillate_count = 0
                    print("To right because oscillate occurs")
                    ru.wheel_turn(left_motor, Direction.FORWARD, 0.5)
                    ru.wheel_turn(right_motor, Direction.BACKWARD, 0.5)
                    # myCreate.step(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 1)
                    myCreate.step(TIMESTEP * 36)
                    # for _ in range(3):
                        # myCreate.step(TIMESTEP * 20)
                # Should go back
                elif cpDot[1][1] > row_line_back:
                    print("To back left (head to right)")
                    last_direction = "back"
                    for _ in range(1):
                        ru.wheel_turn(left_motor, Direction.BACKWARD, 0.5)
                        ru.wheel_turn(right_motor, Direction.BACKWARD, 0.5)
                        myCreate.step(TIMESTEP * 36 // 2)
                    for _ in range(1):
                        ru.wheel_turn(left_motor, Direction.FORWARD, 0.5)
                        ru.wheel_turn(right_motor, Direction.BACKWARD, 0.5)
                        myCreate.step(TIMESTEP * 36)
                # Should go left more
                elif cpDot[1][0] < (image_middle_col-range_accept):
                    print("To left")
                    if last_direction == "right":
                        oscillate_count += 1
                        stuck_count = 0
                    elif last_direction == "left":
                        stuck_count += 1
                    last_direction = "left"
                    ru.wheel_turn(left_motor, Direction.BACKWARD, 0.5)
                    ru.wheel_turn(right_motor, Direction.FORWARD, 0.5)
                    # myCreate.step(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 1)
                    myCreate.step(TIMESTEP * 36 // 2)
                    # print(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 2)
                    ru.wheel_turn(left_motor, Direction.STOP, 0)
                    ru.wheel_turn(right_motor, Direction.STOP, 0)
                    myCreate.step(TIMESTEP)
                # Should go right more
                elif cpDot[1][0] > (image_middle_col+range_accept):
                    print("To right")
                    if last_direction == "left":
                        oscillate_count += 1
                        stuck_count = 0
                    elif last_direction == "right":
                        stuck_count += 1
                    last_direction = "right"
                    ru.wheel_turn(left_motor, Direction.FORWARD, 0.5)
                    ru.wheel_turn(right_motor, Direction.BACKWARD, 0.5)
                    # myCreate.step(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 1)
                    myCreate.step(TIMESTEP * 36 // 2)
                    # print(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 2)
                    ru.wheel_turn(left_motor, Direction.STOP, 0)
                    ru.wheel_turn(right_motor, Direction.STOP, 0)
                    myCreate.step(TIMESTEP)
                else:
                    print("To front")
                    last_direction = "front"
                    ru.wheel_turn(left_motor, Direction.FORWARD, 0.75)
                    ru.wheel_turn(right_motor, Direction.FORWARD, 0.75)
                    myCreate.step(TIMESTEP * 10)
            else:
                last_direction = "back"
                # ru.wheel_turn(left_motor, Direction.STOP, 0)
                # ru.wheel_turn(right_motor, Direction.STOP, 0)
                ru.wheel_turn(left_motor, Direction.FORWARD, 0.5)
                ru.wheel_turn(right_motor, Direction.BACKWARD, 0.5)
                myCreate.step(TIMESTEP * 20)


    cv2.destroyAllWindows()

if __name__ == "__main__":
    initDevices()
    print("[INFO] myCreate all devices initialized successfully!")

    # mainLoop()
    obstacle_avoidance_main_loop()