''' Utility to process the output image from the model
1. A list of (start, end)
2. Find the max of (start - end) in the list
3. Draw a dot in the middle of that range
4. Find controlling point from those middle dots
'''

import cv2
import numpy as np
from sys import exit
from time import sleep

def convertImage(path, outPath):
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    image *= 255
    cv2.imwrite(outPath, image)

def readImage(path):
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image


def findConsecutive(row):
    consecutiveList = []
    start, end = -1, -1
    i = -1
    while i < len(row) - 1:
        i += 1
        # Find the next white pixel
        if row[i] != 255:
            # print(f"\r{i}", end="")
            continue

        start = i
        while i+1 < len(row) and row[i+1] == row[i]:
            i += 1
        end = i
        if end - start >= 160:
            consecutiveList.append((start, end))
    return consecutiveList

def calcMiddle(l, rowIdx):
    if len(l) > 0:
        maxIdx = 0
        maxValue = l[0][1]- l[0][0]

        for i in range(1, len(l)):
            if l[i][1] - l[i][0] > maxValue:
                maxValue = l[i][1] - l[i][0]
                maxIdx = i
    
        middle = l[maxIdx][0] + int(maxValue/2)
        # cv2.circle(middleLineImage, (middle, rowIdx), 3, (0, 255, 0), cv2.FILLED)
        return (middle, rowIdx)
    return None

if __name__ == "__main__":
    # convertImage("cl_0_png.rf.231677ca9a6d2b14b6757127cad860b6_mask.png", "maskImage.png")
    # convertImage("cl_1_png.rf.088f701647b8fb6d7a924ddc3ba6b764_mask.png", "maskImage2.png")
    # convertImage("train_006_mask.png", "maskImage2.png")

    # image = readImage("maskImage.png")
    # image = readImage("maskImage2.png")
    image = readImage("/home/jordenhuang/programs/lab246/yolov5/road/image/road_mask.png")
    # cv2.imshow("window", image)
    # cv2.waitKey(0)

    image = cv2.resize(image, (256, 256))

    # create empty numpy array
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

    # Draw line between dots
    # for p1, p2 in zip(dots, dots[1:]):
    #     cv2.line(middleLineImage, p1, p2, (0, 255, 0), 2)

    # Draw the dots
    for dot in dots:
        cv2.circle(middleLineImage, dot, 2, (0, 255, 0), cv2.FILLED)
    
    # Find the controlling point
    cp = []
    for dot in dots:
        if dot[1] > 77: # Let the controlling point set between row 78 and 254
            cp.append([abs(int(image.shape[1] / 2 - dot[0])), dot])
    if len(cp) != 0:
        cpDot = max(cp, key=lambda x: x[0])
        # Draw the controlling point
        cv2.circle(middleLineImage, cpDot[1], 3, (255, 0, 255), cv2.FILLED)
        print(f"Controlling point: {cpDot[1]}, distance from middle: {cpDot[0]}")

        # Display the result
        # cv2.imshow("window", image)
        cv2.imshow("result", middleLineImage)
        while cv2.waitKey(0) != ord('q'):
            sleep(0.5)
        cv2.destroyAllWindows()
    
