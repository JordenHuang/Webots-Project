'''
TODO: Plot the middle points and the controlling point
'''
from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard

from util import findConsecutive, calcMiddle

PI = 3.14159
WHEEL_MAX_SPEED = 6.28
FORWARD_RATIO = 0.5
TURN_RATIO = 0.85

mannual_control = False

# create the Robot instance.
turtlebot = Robot()

# get the time step of the current world.
TIMESTEP = int(turtlebot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(TIMESTEP)

# Cameras
camera_left = turtlebot.getDevice("camera_left")
camera_right = turtlebot.getDevice("camera_right")
camera_left.enable(TIMESTEP)
camera_right.enable(TIMESTEP)

# Wheels
wheel_left = turtlebot.getDevice("left_wheel_motor")
wheel_right = turtlebot.getDevice("right_wheel_motor")
wheel_left.setPosition(float('+inf'));
wheel_right.setPosition(float('+inf'));
wheel_left.setVelocity(0.0)
wheel_right.setVelocity(0.0)


# Main loop
counter = 0
while turtlebot.step(TIMESTEP) != -1:
    key = keyboard.getKey()
    if key == ord('M'):
        mannual_control = not mannual_control

    if mannual_control == True:
        # TurtleBot movement
        if key == ord('W'):
            wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        elif key == ord('S'):
            wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif key == ord('A'):
            wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif key == ord('D'):
            wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        else:
            wheel_left.setVelocity(0.0)
            wheel_right.setVelocity(0.0)
    else:
        counter += 1

        if counter % 100 == 0:
            # camera_left.saveImage(f"images/cl_{idx}.png", None)
            # camera_right.saveImage(f"images/cr_{idx}.png", None)
            camImage = camera_left.getImage()

            # Semantic segmentation
            # model.predict(camImage)
            image = None

            # Find controlling point
            dots = []

            # Get all the middle points
            for rowIdx in range(len(image)):
                l = findConsecutive(image[rowIdx])
                # print(l)
                dot = calcMiddle(l)
                if dot:
                    dots.append(dot)

            # Find the controlling point
            cp = []
            for dot in dots:
                if dot[1] > 77: # Let the controlling point set between row 78 and 254
                    cp.append([abs(int(image.shape[1] / 2 - dot[0])), dot])

            cpDot = max(cp, key=lambda x: x[0])
            print(f"Controlling point: {cpDot[1]}, distance from middle: {cpDot[0]}")

            image_middle_col = image.shape[1]/2
            turn_ratio = cpDot[0] / image_middle_col
            # Should go left more
            if cpDot[1][0] < image_middle_col:
                wheel_left.setVelocity(WHEEL_MAX_SPEED * -(turn_ratio))
                wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO * 0.5)
            # Should go right more
            elif cpDot[1][0] > image_middle_col:
                wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO * 0.5)
                wheel_right.setVelocity(WHEEL_MAX_SPEED * -(turn_ratio))
            else:
                wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO * 0.5)
                wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO * 0.5)
        else:
            wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)


