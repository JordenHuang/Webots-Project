from controller import Robot
from controller import wb, PositionSensor
from controller import Supervisor, Node
from controller import Keyboard
from controller.pen import Pen

import cv2
import numpy as np

from direction import Direction
import robot_util as ru
from my_model import *
from abstacle_avoidance_algorithm import *

mannual_control = False
counter = -1
flag = False
fl = True

last_direction = None
oscillate_count = 0

stuck_count = 0

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

# Pen
pen = turtlebot.getDevice('pen')
pen.setInkColor(0x555555, 0.1)
pen.write(False)

# Main loop
while turtlebot.step(TIMESTEP) != -1:
    counter += 1
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
        range_accept = 3 #80
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
            print(counter)
        else:
            print("cpDot is none")
        cv2.imshow("camera right", cv2.cvtColor(camImage, cv2.COLOR_RGB2BGR))
        cv2.imshow("result", cv2.cvtColor(middleLineImage, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    if mannual_control == True:
        # TurtleBot movement
        if key == ord('W'):
            ru.wheel_turn(wheel_left, Direction.FORWARD, 1)
            ru.wheel_turn(wheel_right, Direction.FORWARD, 1)
        elif key == ord('S'):
            ru.wheel_turn(wheel_left, Direction.BACKWARD, 1)
            ru.wheel_turn(wheel_right, Direction.BACKWARD, 1)
        elif key == ord('A'):
            ru.wheel_turn(wheel_left, Direction.FORWARD, 1)
            ru.wheel_turn(wheel_right, Direction.BACKWARD, 1)
        elif key == ord('D'):
            ru.wheel_turn(wheel_left, Direction.BACKWARD, 1)
            ru.wheel_turn(wheel_right, Direction.FORWARD, 1)
        else:
            ru.wheel_turn(wheel_left, Direction.STOP, 0)
            ru.wheel_turn(wheel_right, Direction.STOP, 0)
        # if fl:
        #     fl = False
        #     ru.wheel_turn(wheel_left, Direction.FORWARD, 0.5)
        #     ru.wheel_turn(wheel_right, Direction.BACKWARD, 0.5)
        #     turtlebot.step(TIMESTEP*72)
        #     turtlebot.step(TIMESTEP*72)
        #     turtlebot.step(TIMESTEP*72)
        #     turtlebot.step(TIMESTEP*72)
        # else:
        #     ru.wheel_turn(wheel_left, Direction.STOP, 0)
        #     ru.wheel_turn(wheel_right, Direction.STOP, 0)

    else:
        if cpDot:
            # Stuck detection
            if stuck_count > 6:
                stuck_count = 0
                print("To back because it stucks")
                ru.wheel_turn(wheel_left, Direction.BACKWARD, 0.55)
                ru.wheel_turn(wheel_right, Direction.BACKWARD, 0.55)
                for _ in range(3):
                    turtlebot.step(TIMESTEP * 20)
            # Oscillate detection
            elif oscillate_count > 4:
                oscillate_count = 0
                print("To right because oscillate occurs")
                ru.wheel_turn(wheel_left, Direction.BACKWARD, 0.5)
                ru.wheel_turn(wheel_right, Direction.FORWARD, 0.5)
                # turtlebot.step(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 1)
                turtlebot.step(TIMESTEP * 36)
                # for _ in range(3):
                    # turtlebot.step(TIMESTEP * 20)
            # Should go back
            elif cpDot[1][1] > row_line_back:
                print("To back left (head to right)")
                last_direction = "back"
                for _ in range(1):
                    ru.wheel_turn(wheel_left, Direction.BACKWARD, 0.5)
                    ru.wheel_turn(wheel_right, Direction.BACKWARD, 0.5)
                    turtlebot.step(TIMESTEP * 36 // 2)
                for _ in range(1):
                    ru.wheel_turn(wheel_left, Direction.BACKWARD, 0.5)
                    ru.wheel_turn(wheel_right, Direction.FORWARD, 0.5)
                    turtlebot.step(TIMESTEP * 36)
            # Should go left more
            elif cpDot[1][0] < (image_middle_col-range_accept):
                print("To left")
                if last_direction == "right":
                    oscillate_count += 1
                    stuck_count = 0
                elif last_direction == "left":
                    stuck_count += 1
                last_direction = "left"
                ru.wheel_turn(wheel_left, Direction.FORWARD, 0.5)
                ru.wheel_turn(wheel_right, Direction.BACKWARD, 0.5)
                # turtlebot.step(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 1)
                turtlebot.step(TIMESTEP * 36 // 2)
                # print(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 2)
                ru.wheel_turn(wheel_left, Direction.STOP, 0)
                ru.wheel_turn(wheel_right, Direction.STOP, 0)
                turtlebot.step(TIMESTEP)
            # Should go right more
            elif cpDot[1][0] > (image_middle_col+range_accept):
                print("To right")
                if last_direction == "left":
                    oscillate_count += 1
                    stuck_count = 0
                elif last_direction == "right":
                    stuck_count += 1
                last_direction = "right"
                ru.wheel_turn(wheel_left, Direction.BACKWARD, 0.5)
                ru.wheel_turn(wheel_right, Direction.FORWARD, 0.5)
                # turtlebot.step(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 1)
                turtlebot.step(TIMESTEP * 36 // 2)
                # print(TIMESTEP * 30 * (abs(cpDot[1][0]-image_middle_col)) // image_middle_col // 2)
                ru.wheel_turn(wheel_left, Direction.STOP, 0)
                ru.wheel_turn(wheel_right, Direction.STOP, 0)
                turtlebot.step(TIMESTEP)
            else:
                print("To front")
                last_direction = "front"
                ru.wheel_turn(wheel_left, Direction.FORWARD, 0.75)
                ru.wheel_turn(wheel_right, Direction.FORWARD, 0.75)
                turtlebot.step(TIMESTEP * 10)
        else:
            last_direction = "back"
            # ru.wheel_turn(wheel_left, Direction.STOP, 0)
            # ru.wheel_turn(wheel_right, Direction.STOP, 0)
            ru.wheel_turn(wheel_left, Direction.BACKWARD, 0.5)
            ru.wheel_turn(wheel_right, Direction.FORWARD, 0.5)
            turtlebot.step(TIMESTEP * 20)


cv2.destroyAllWindows()