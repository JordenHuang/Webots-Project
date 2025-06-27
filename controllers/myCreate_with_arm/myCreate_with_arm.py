from controller import Robot
from controller import Keyboard
import cv2
import numpy as np


# Create the robot instance
myCreate = Robot()

# get the time step of the current world.
TIMESTEP = int(myCreate.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(TIMESTEP)

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

# MyCobot
joints = []
for i in range(6):
    joints.append(myCreate.getDevice(f"joint{i}_rotational_motor"))

PI = 3.14159
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
        myCreate.step(TIMESTEP)  # Small delay to control speed

degrees = [0, -135, 150, -125, 90, 0]
mycobot_send_angles(degrees)

counter = 0
x_pressed = False
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