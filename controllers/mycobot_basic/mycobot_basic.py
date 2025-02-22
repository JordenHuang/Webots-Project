# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
# import math
from time import sleep

PI = 3.14159

# create the Robot instance.
myCobot = Robot()
# myCobotSup = Supervisor()

# get the time step of the current world.
timestep = int(myCobot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# myCobot.getDevice("joint2_to_joint1").setPosition(3.14)
joints = []
for i in range(6):
    joints.append(myCobot.getDevice(f"joint{i}_rotational_motor"))

# contact_joints = []


def mycobot_send_angles(degrees: list):
    rad = [d * (PI / 180) for d in degrees]
    print(rad)
    for i in range(len(joints)):
        joints[i].setPosition(rad[i])

    sleep(0.5)


''' Joint angle constrants
J1: -168 ~ 168 (2.9321506666666663)
J2:	-135 ~ 135 (2.3561924999999997)
J3:	-150 ~ 150 (2.6179916666666667)
J4:	-145 ~ 145 (2.530725277777778)
J5:	-165 ~ 165 (2.8797908333333333)
J6:	-180 ~ 180 (3.14159)
'''


# Main loop:
# - perform simulation steps until Webots is stopping the controller
print('Press 0, 1, 2 or 3 to change modes')
d = [1] * 6
to_move = False
mode = 0
while myCobot.step(timestep) != -1:
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


    if mode == 1:
        degrees = [0, 0, 0, 0, 0, 0]
        mycobot_send_angles(degrees)
        mode = 0

    elif mode == 2:
        degrees = [0, -135, 150, -145, 90, 0]
        mycobot_send_angles(degrees)
        mode = 0

    elif mode == 3:
        for i in range(1, len(joints)):
            t = joints[i].getTargetPosition()
            st = 0.025
            if d[i] == 1:
                t += st
            else:
                t -= st
            if (t > joints[i].getMaxPosition()):
                t -= st
                d[i] *= -1
            elif (t < joints[i].getMinPosition()):
                t += st
                d[i] *= -1
            joints[i].setPosition(t)


# Enter here exit cleanup code.
