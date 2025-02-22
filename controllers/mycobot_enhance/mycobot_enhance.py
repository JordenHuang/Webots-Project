# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
# import math
import time

PI = 3.14159

# create the Robot instance.
myCobot = Robot()

# get the time step of the current world.
TIMESTEP = int(myCobot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(TIMESTEP)

joints = []
for i in range(6):
    joints.append(myCobot.getDevice(f"joint{i}_rotational_motor"))

gripper_left = myCobot.getDevice("gripper_left_arm_rotational_motor")
gripper_right = myCobot.getDevice("gripper_right_arm_rotational_motor")


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
        myCobot.step(TIMESTEP)  # Small delay to control speed

''' Joint angle constrants
J1: -168 ~ 168 (2.9321506666666663)
J2:	-135 ~ 135 (2.3561924999999997)
J3:	-150 ~ 150 (2.6179916666666667)
J4:	-145 ~ 145 (2.530725277777778)
J5:	-165 ~ 165 (2.8797908333333333)
J6:	-180 ~ 180 (3.14159)
'''

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
mode = 0
degrees = [0, -135, 150, -120, 90, 0]
mycobot_send_angles(degrees)
while myCobot.step(TIMESTEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    key = keyboard.getKey()

    if key == ord('0'):
        print('Change to mode 0')
        mode = 0
    elif key == ord('1'):
        print('Change to mode 1')
        mode = 1
    elif key == ord('2'):
        print('Change to mode 2')
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


    if mode == 1:
        degrees = [0, 0, 0, 0, 165, 0]
        mycobot_send_angles(degrees)
        mode = 0
    elif mode == 2:
        degrees = [0, -135, 150, -125, 90, 0]
        mycobot_send_angles(degrees)
        mode = 0
    elif mode == 3:
        # Without gripper
        # degrees = [0, 127, -5, -25, 165, 0]
        # Withgripper
        # degrees = [0, 100, 15, -25, 165, 0]
        # Near target
        degrees = [0, 110, 5, -15, 165, 0]
        mycobot_send_angles(degrees)
        mode = 0

    elif mode == 4:
        deg = 10
        gripper_left.setPosition(deg * PI / 180)
        gripper_right.setPosition(deg * PI / 180)
        mode = 0
    elif mode == 5:
        deg = 0
        gripper_left.setPosition(deg * PI / 180)
        gripper_right.setPosition(deg * PI / 180)
        degrees = [0, 127, -5, -25, 165, 0]
        mode = 0
    
    elif mode == 6:
        degrees = [0, 80, 15, -25, 165, 0]
        mycobot_send_angles(degrees)
        mode = 0




# Enter here exit cleanup code.
