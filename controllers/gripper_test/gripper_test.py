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

gripper_right = myCobot.getDevice("gripper_right::base_outer::rotational_motor")
# gripper_left = myCobot.getDevice("gripper_left_arm_rotational_motor")

gripper_right_inner = myCobot.getDevice("gripper_right::base_inner::rotational_motor")
gripper_right_paddle_inner = myCobot.getDevice("gripper_right::paddle::rotational_motor")


# Main loop:
# - perform simulation steps until Webots is stopping the controller
mode = 0
while myCobot.step(TIMESTEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    key = keyboard.getKey()

    if key == ord('0'):
        print('Change to mode 0 (gripper)')
        mode = 0
    elif key == ord('1'):
        print('Change to mode 1 (gripper)')
        mode = 1
    elif key == ord('2'):
        print('Change to mode 2 (gripper)')
        mode = 2

    if mode == 1:
        deg = 10
        # gripper_left.setPosition(deg * PI / 180)
        gripper_right.setPosition(deg * PI / 180)
        # gripper_right_inner.setPosition(deg * PI / 180)
        # gripper_right_paddle_inner.setPosition(deg*3 * PI / 180)
        mode = 0
    elif mode == 2:
        deg = -10
        # gripper_left.setPosition(deg * PI / 180)
        gripper_right.setPosition(deg * PI / 180)
        degrees = [0, 127, -5, -25, 165, 0]
        mode = 0



# Enter here exit cleanup code.
