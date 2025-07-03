from controller import Robot
from controller import Keyboard
import sys
import cv2
import numpy as np
import socket
from random import randint

from grid import Grid


# Create the robot instance
myCreate = Robot()

# get the time step of the current world.
TIMESTEP = int(myCreate.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIMESTEP)

# ========== MyCreate ==========
MAX_SPEED  = 16
NULL_SPEED = 0
HALF_SPEED = 8
MIN_SPEED  = -16

WHEEL_RADIUS       = 0.031
AXLE_LENGTH        = 0.271756 # 0.235
ENCODER_RESOLUTION = 507.9188

TO_LEFT  = -np.pi / 2
TO_RIGHT = np.pi / 2
GO_FRONT = 0.1
GO_BACK  = -0.1

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

bumpers = {
    "bumper_left" : None,
    "bumper_right": None,
}
for name in bumpers.keys():
    bumpers[name] = myCreate.getDevice(name)
    bumpers[name].enable(TIMESTEP)
def is_there_a_collision_at_left() -> bool:
    return bumpers["bumper_left"].getValue() != 0.0

def is_there_a_collision_at_right() -> bool:
    return bumpers["bumper_right"].getValue() != 0.0

def step():
    if myCreate.step(TIMESTEP) == -1:
        sys.exit(0)

def stop():
    left_motor.setVelocity(-NULL_SPEED)
    right_motor.setVelocity(-NULL_SPEED)

def turn(angle: float):
    l_offset = left_position_sensor.getValue()
    r_offset = right_position_sensor.getValue()
    neg = -1.0 if angle < 0.0 else 1.0
    vel = neg * np.pi * 1
    left_motor.setVelocity(vel)
    right_motor.setVelocity(-vel)
    while True:
        step()
        l = left_position_sensor.getValue() - l_offset
        r = right_position_sensor.getValue() - r_offset
        dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
        dr = r * WHEEL_RADIUS                        # distance covered by right wheel in meter
        orientation = neg * (dl - dr) / AXLE_LENGTH  # delta orientation in radian
        # print(orientation, angle)
        # 希望多轉一點
        if abs(angle) - abs(orientation) < -0.08:
            break
    stop()
    step()

# Turn 90 degrees
def my_turn(angle: float):
    neg = -1.0 if angle < 0.0 else 1.0
    vel = neg * np.pi * 1
    left_motor.setVelocity(vel)
    right_motor.setVelocity(-vel)
    myCreate.step(TIMESTEP * 74)
    # stop()
    # i = (np.pi * AXLE_LENGTH / 2) / (WHEEL_RADIUS * np.pi * 2)
    # neg = -1.0 if angle < 0.0 else 1.0
    # print(i)
    # left_motor.setVelocity(neg * np.pi * 2)
    # right_motor.setVelocity(-neg * np.pi * 2)
    # c = 0
    # while c <= i:
    #     myCreate.step(TIMESTEP)
    #     c += 1 / TIMESTEP
    # stop()
    return

def go(distance_in_m: float):
    # print(f"Go {distance_in_m}m")
    l_offset = left_position_sensor.getValue()
    neg = -1.0 if distance_in_m < 0.0 else 1.0
    vel = neg * np.pi * 1
    left_motor.setVelocity(vel)
    right_motor.setVelocity(vel)
    flag = False
    while True:
        step()
        l = left_position_sensor.getValue() - l_offset
        dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
        # print(f"dl: {dl}")
        if abs(distance_in_m) - abs(dl) <= 0.001:
            break

        if is_there_a_collision_at_left() or is_there_a_collision_at_right():
            print("[bump] collision")
            flag = True
            break
    stop()
    step()

    if flag:
        distance_in_m = dl
        left_motor.setVelocity(-vel)
        right_motor.setVelocity(-vel)
        l_offset = left_position_sensor.getValue()
        while True:
            step()
            l = left_position_sensor.getValue() - l_offset
            dl = l * WHEEL_RADIUS                        # distance covered by left wheel in meter
            # print(f"dl: {dl}")
            if abs(distance_in_m) - abs(dl) <= 0.001:
                break
        stop()
        step()
        return -1
    else:
        return 1


# ========== MyCobot ==========
# joints = []
# for i in range(6):
#     joints.append(myCreate.getDevice(f"joint{i}_rotational_motor"))

# PI = 3.14159
# def mycobot_send_angles(degrees: list, speed=0.05):
#     rad = [d * (PI / 180) for d in degrees]  # Convert degrees to radians
#     current_angles = [j.getTargetPosition() for j in joints]  # Get current joint angles

#     while any(abs(curr - target) > 0.01 for curr, target in zip(current_angles, rad)):  # Loop until close enough
#         for i in range(len(joints)):
#             diff = rad[i] - current_angles[i]
#             step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
#             current_angles[i] += step * (1 if diff > 0 else -1)  # Move in the correct direction
#             joints[i].setPosition(current_angles[i])  # Apply new position

#         # time.sleep(0.01)  # Control loop timing
#         myCreate.step(TIMESTEP)  # Small delay to control speed

# ==============================

# degrees = [0, -135, 150, -125, 90, 0]
# mycobot_send_angles(degrees)


print("Try to connect to server...")
clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientsocket.connect(('localhost', 8989))
print("Connected")

grid = Grid(0.1)

counter = 0
back_counter = 0
update = True
# Main loop
while myCreate.step(TIMESTEP) != -1:
    WHEEL_MAX_SPEED = 6.28
    FORWARD_RATIO = 1 # 0.75

    update = True
    key = keyboard.getKey()
    if key == ord('W'):
        go(GO_FRONT)
        grid.moveForward()
        grid.displayGrid()
        print("GO_FRONT")
    elif key == ord('S'):
        go(GO_BACK)
        grid.moveBackward()
        grid.displayGrid()
        print("GO_BACK")
    elif key == ord('A'):
        # turn(TO_LEFT)
        my_turn(TO_LEFT)
        grid.turnLeft()
        grid.displayGrid()
        print("TO_LEFT")
    elif key == ord('D'):
        # turn(TO_RIGHT)
        my_turn(TO_RIGHT)
        grid.turnRight()
        grid.displayGrid()
        print("TO_RIGHT")
    else:
        # update = False
        stop()

    if update == True:
        update = False
        camRawLeft  = camera_left.getImage()  # returns a byte string
        width = camera_left.getWidth()
        height = camera_left.getHeight()
        # Convert to a NumPy array, reshape, and convert RGB to BGR
        image_np = np.frombuffer(camRawLeft, dtype=np.uint8).reshape((height, width, 4))  # RGBA
        camImageLeft = image_np[:, :, :3]  # Drop alpha channel
        camImageLeft = cv2.cvtColor(camImageLeft, cv2.COLOR_BGR2RGB)

        _, buffer = cv2.imencode(".jpg", camImageLeft)
        buffer_bytes = buffer.tobytes()
        buffer_length = len(buffer_bytes)
        buffer_length_bytes = buffer_length.to_bytes(10, 'big')
        clientsocket.sendall(buffer_length_bytes + buffer_bytes)

        # print("wait recv", flush=True)
        cmd_length_bytes = clientsocket.recv(10)
        cmd_length = int.from_bytes(cmd_length_bytes, 'big')
        # print(f"cmd length: {cmd_length}", flush=True)
        cmd_bytes = b''
        while len(cmd_bytes) < cmd_length:
            packet = clientsocket.recv(cmd_length - len(cmd_bytes))
            if not packet:
                break
            cmd_bytes += packet
        cmd = cmd_bytes.decode()
        print(f"Receive cmd: '{cmd}'", flush=True)

        if cmd == "front":
            # if grid.getFront() == grid.GRID_HAS_OBSTACLE:
            #     cmd = "back"
            # else:
            if True:
                r = go(GO_FRONT)
                grid.moveForward()
                grid.displayGrid()
                print("GO_FRONT")
                if r == -1:
                    grid.moveBackward()
                    grid.markPrevious(grid.GRID_HAS_OBSTACLE)
                    cmd = "back"
        if cmd == "back":
            # if randint(0, 99) < 30:
            #     go(GO_BACK)
            #     grid.moveBackward()
            #     grid.displayGrid()
            #     print("GO_BACK")
            #     turn(TO_RIGHT)
            # if randint(0, 99) < 50:
            my_turn(TO_RIGHT)
            grid.turnRight()
            grid.displayGrid()
            print("TO_RIGHT")
            # else:
            #     my_turn(TO_LEFT)
            #     grid.turnLeft()
            #     grid.displayGrid()
            #     print("TO_LEFT")
        elif cmd == "left":
            # turn(TO_LEFT)
            my_turn(TO_LEFT)
            # grid.markFront(grid.GRID_HAS_OBSTACLE)
            grid.turnLeft()
            grid.displayGrid()
            print("TO_LEFT")
        elif cmd == "right":
            # turn(TO_RIGHT)
            my_turn(TO_RIGHT)
            # grid.markFront(grid.GRID_HAS_OBSTACLE)
            grid.turnRight()
            grid.displayGrid()
            print("TO_RIGHT")
        else:
            stop()


    # camRawLeft  = camera_left.getImage()  # returns a byte string
    # camRawRight = camera_right.getImage()  # returns a byte string
    # width = camera_left.getWidth()
    # height = camera_left.getHeight()

    # # Convert to a NumPy array, reshape, and convert RGB to BGR
    # image_np = np.frombuffer(camRawLeft, dtype=np.uint8).reshape((height, width, 4))  # RGBA
    # camImageLeft = image_np[:, :, :3]  # Drop alpha channel
    # # camImageLeft = cv2.cvtColor(camImageLeft, cv2.COLOR_BGR2RGB)
    # image_np2 = np.frombuffer(camRawRight, dtype=np.uint8).reshape((height, width, 4))  # RGBA
    # camImageRight = image_np2[:, :, :3]  # Drop alpha channel
    # # camImageRight = cv2.cvtColor(camImageRight, cv2.COLOR_BGR2RGB)

    # cv2.imshow('Left' , camImageLeft)
    # cv2.imshow('Right', camImageRight)
    # cv2.waitKey(1)

    # gray_left = cv2.cvtColor(camImageLeft, cv2.COLOR_BGR2GRAY)
    # gray_right = cv2.cvtColor(camImageRight, cv2.COLOR_BGR2GRAY)

    # stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)
    # disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    # cv2.imshow('disp', disparity)

    # Y = 118 / (189 - 133)
    # # print(camera_left.getFocalDistance())
    # b = 0.1 # m
    # f = 320 # 焦距
    # # depth = b*f / (disparity+1e-6)
    # depth = b*f / (242 - 137)
    # print(depth)

cv2.destroyAllWindows()
