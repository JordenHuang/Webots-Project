'''
TODO: Plot the middle points and the controlling point
'''
from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard

import cv2
import numpy as np

from util import findConsecutive, calcMiddle
from pred import model, predict_image

PI = 3.14159
WHEEL_MAX_SPEED = 6.28
FORWARD_RATIO = 1 # 0.75
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
counter = -1
flag = False
while turtlebot.step(TIMESTEP) != -1:
    key = keyboard.getKey()
    if key == ord('M'):
        if not flag:
            if not mannual_control: print("Switch mode --> Mannual")
            else: print("Switch mode --> Obstacle avoidence")
            mannual_control = not mannual_control
            flag = True
    else:
        flag = False

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

        if counter % 80 == 0:
            # camera_left.saveImage(f"images/cl_{idx}.png", None)
            # camera_right.saveImage(f"images/cr_{idx}.png", None)

            raw = camera_left.getImage()  # returns a byte string

            width = camera_left.getWidth()
            height = camera_left.getHeight()

            # Convert to a NumPy array, reshape, and convert RGB to BGR
            image_np = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 4))  # RGBA
            camImage = image_np[:, :, :3]  # Drop alpha channel
            camImage = cv2.cvtColor(camImage, cv2.COLOR_BGR2RGB)

            # Semantic segmentation
            image = predict_image(model, None, camImage, False)


            # Find controlling point
            middleLineImage = np.copy(image)
            middleLineImage = cv2.cvtColor(middleLineImage, cv2.COLOR_GRAY2BGR)
            dots = []

            # Get all the middle points
            for rowIdx in range(len(image)):
                l = findConsecutive(image[rowIdx])
                # print(l)
                dot = calcMiddle(l, rowIdx)
                if dot:
                    dots.append(dot)

            # Some parameters
            row_line_ok = 170# 128 + 32
            row_line_back = 190
            range_accept = 3#80
            image_middle_col = int(image.shape[1]/2)
            # turn_ratio = cpDot[0] / (image_middle_col/1.25-range_accept)
            turn_ratio = TURN_RATIO / 2

            # Find the controlling point
            cp = []
            for dot in dots:
                if dot[1] > row_line_ok:# and dot[1] < 253: # Let the controlling point set between row 78 and 252
                    cp.append([abs(int(image.shape[1] / 2 - dot[0])), dot])

            if len(cp) != 0:
                cpDot = max(cp, key=lambda x: x[0])
                print(f"Controlling point: {cpDot[1]}, distance from middle: {cpDot[0]}")

                # Draw the dots
                for dot in dots:
                    cv2.circle(middleLineImage, dot, 2, (0, 255, 0), cv2.FILLED)
                # Draw the controlling point
                cv2.circle(middleLineImage, cpDot[1], 3, (255, 0, 255), cv2.FILLED)
                # Draw the row line
                cv2.line(middleLineImage, (0, row_line_ok), (middleLineImage.shape[1], row_line_ok), (0, 0, 255), 1)
                cv2.line(middleLineImage, (0, row_line_back), (middleLineImage.shape[1], row_line_back), (0, 0, 127), 1)
                cv2.line(middleLineImage, (image_middle_col-range_accept, 0), (image_middle_col-range_accept, middleLineImage.shape[0]), (0, 0, 127), 1)
                cv2.line(middleLineImage, (image_middle_col+range_accept, 0), (image_middle_col+range_accept, middleLineImage.shape[0]), (0, 0, 127), 1)
                # print(camImage.shape, middleLineImage.shape)
                cv2.imshow("camera", cv2.cvtColor(camImage, cv2.COLOR_RGB2BGR))
                cv2.imshow("result", middleLineImage)
                cv2.waitKey(1)

                # Should go back
                if cpDot[1][1] > row_line_back:
                    print("To back left (head to right)")
                    # for _ in range(1):
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO / 5)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO / 5)
                        # wheel_left.setVelocity(WHEEL_MAX_SPEED * (turn_ratio))
                        # wheel_right.setVelocity(WHEEL_MAX_SPEED * -(turn_ratio))
                        # turtlebot.step(TIMESTEP)
                # Should go left more
                elif cpDot[1][0] < (image_middle_col-range_accept):
                    print("To left")
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * -(turn_ratio))
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO / 5)
                # Should go right more
                elif cpDot[1][0] > (image_middle_col+range_accept):
                    print("To right")
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO / 5)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -(turn_ratio))
                else:
                    print("To front")
                    wheel_left.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                    wheel_right.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            else:
                # wheel_left.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO * 0.5)
                # wheel_right.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO * 0.5)
                pass


cv2.destroyAllWindows()