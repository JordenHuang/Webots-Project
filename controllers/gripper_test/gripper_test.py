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

gripper_right_base_outer = myCobot.getDevice("gripper_right::base_outer::rotational_motor")
gripper_right_outer_paddle = myCobot.getDevice( "gripper_right::outer_paddle::rotational_motor")
gripper_right_inner = myCobot.getDevice("gripper_right::base_inner::rotational_motor")

gripper_left_base_outer = myCobot.getDevice("gripper_left::base_outer::rotational_motor")
gripper_left_outer_paddle = myCobot.getDevice( "gripper_left::outer_paddle::rotational_motor")
gripper_left_inner = myCobot.getDevice("gripper_left::base_inner::rotational_motor")


def mycobot_gripper_send_angle(degree: int, speed=0.01):
    rad_right = degree * (PI / 180)  # Convert degrees to radians
    current_angle_right = gripper_right_base_outer.getTargetPosition()  # Get current joint angles

    rad_left = (-degree) * (PI / 180)  # Convert degrees to radians
    current_angle_left = gripper_left_base_outer.getTargetPosition()  # Get current joint angles

    flag = False
    while True:  # Loop until close enough
        flag = False
        # Right
        if abs(current_angle_right - rad_right) > 0.01:
            diff = rad_right - current_angle_right
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angle_right += step * (1 if diff > 0 else -1)  # Move in the correct direction

            gripper_right_base_outer.setPosition(current_angle_right)  # Apply new position
            gripper_right_outer_paddle.setPosition(-current_angle_right)  # Apply new position
            gripper_right_inner.setPosition(current_angle_right)  # Apply new position
        else:
            flag = True

        # Left
        if abs(current_angle_left - rad_left) > 0.01:
            diff = rad_left - current_angle_left
            step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
            current_angle_left += step * (1 if diff > 0 else -1)  # Move in the correct direction

            gripper_left_base_outer.setPosition(current_angle_left)  # Apply new position
            gripper_left_outer_paddle.setPosition(-current_angle_left)  # Apply new position
            gripper_left_inner.setPosition(current_angle_left)  # Apply new position
        else:
            if flag == True:
                break
        
        myCobot.step(TIMESTEP)  # Small delay to control speed



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
        mycobot_gripper_send_angle(deg)
        angle = deg * PI / 180

        # gripper_right_base_outer.setPosition(angle)
        # gripper_right_outer_paddle.setPosition(-angle)
        # gripper_right_inner.setPosition(angle)

        deg = -10
        angle = deg * PI / 180
        # gripper_left_base_outer.setPosition(angle)
        # gripper_left_outer_paddle.setPosition(-angle)
        # gripper_left_inner.setPosition(angle)
        mode = 0
    elif mode == 2:
        deg = -10
        mycobot_gripper_send_angle(deg)
        angle = deg * PI / 180
        # gripper_right_base_outer.setPosition(angle)
        # gripper_right_outer_paddle.setPosition(-angle)
        # gripper_right_inner.setPosition(angle)

        deg = 10
        angle = deg * PI / 180
        # gripper_left_base_outer.setPosition(angle)
        # gripper_left_outer_paddle.setPosition(-angle)
        # gripper_left_inner.setPosition(angle)

        degrees = [0, 127, -5, -25, 165, 0]
        mode = 0



# Enter here exit cleanup code.
