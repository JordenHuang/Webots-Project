''' Utility to process the output image from the model
1. A list of (start, end)
2. Find the max of (start - end) in the list
3. Draw a dot in the middle of that range
4. Find controlling point from those middle dots
'''

import cv2
import numpy as np

def convertMaskToImage(path, outPath):
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    image *= 255
    cv2.imwrite(outPath, image)

def readGrayscaleImage(path):
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image


def findConsecutiveInRow(row):
    consecutiveList = []
    start, end = -1, -1
    i = -1
    while i < len(row) - 1:
        i += 1
        # Find the next white pixel
        if row[i] != 255:
            continue

        start = i
        while i+1 < len(row) and row[i+1] == row[i]:
            i += 1
        end = i
        # (start, end)
        if end - start >= 150:
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

        middle = l[maxIdx][0] + maxValue // 2
        # (col, row)
        return (middle, rowIdx)
    return None

def findControlPoint(imageCols, middleDots, rowLineFront):
    # Find the controlling point
    cp = []
    for dot in middleDots:
        if dot[1] > rowLineFront: # Let the controlling point set between row 78 and 254
            # [distance from middle column, dot]
            distanceFromMiddle = abs(imageCols // 2 - dot[0])
            cp.append([distanceFromMiddle, dot])
    if len(cp) != 0:
        sortedCp = sorted(cp, key=lambda x: x[0])
        # sortedCp = sortedCp[int(len(sortedCp)*0.1):int(len(sortedCp)*0.9)]
        sortedCp = sortedCp[int(len(sortedCp)*0.05):int(len(sortedCp)*0.95)]
        # sortedCp = sortedCp[1:-1]
        if len(sortedCp) != 0:
            cpDot = max(sortedCp, key=lambda x: x[0])
            return cpDot
    return None

def getControlPoint(image, rowLineFront):
    middleDots = []
    # Get all the middle points
    for rowIdx in range(0, len(image), 5):
        l = findConsecutiveInRow(image[rowIdx])
        # print(l)
        dot = calcMiddle(l, rowIdx)
        if dot:
            middleDots.append(dot)

    cpDot = findControlPoint(image.shape[1], middleDots, rowLineFront)
    return cpDot, middleDots


if __name__ == "__main__":
    image = readGrayscaleImage("testMask.png")

    # create empty numpy array
    middleLineImage = np.copy(image)
    middleLineImage = cv2.cvtColor(middleLineImage, cv2.COLOR_GRAY2BGR)
    middleDots = []

    # Get all the middle points
    for rowIdx in range(len(image)):
        l = findConsecutiveInRow(image[rowIdx])
        # print(l)
        dot = calcMiddle(l, rowIdx)
        if dot:
            middleDots.append(dot)

    # Draw line between dots
    # for p1, p2 in zip(dots, dots[1:]):
    #     cv2.line(middleLineImage, p1, p2, (0, 255, 0), 2)

    # Draw the dots
    for dot in middleDots:
        cv2.circle(middleLineImage, dot, 2, (0, 255, 0), cv2.FILLED)
    
    cpDot = findControlPoint(image.shape[1], middleDots)
    cv2.circle(middleLineImage, cpDot[1], 3, (255, 0, 255), cv2.FILLED)
    print(f"Controlling point: {cpDot[1]}, distance from middle: {cpDot[0]}")
    cv2.imshow("result", middleLineImage)
    cv2.waitKey(0)
    
    # Find the controlling point
    # cp = []
    # for dot in dots:
    #     if dot[1] > 77: # Let the controlling point set between row 78 and 254
    #         cp.append([abs(int(image.shape[1] / 2 - dot[0])), dot])
    # if len(cp) != 0:
    #     cpDot = max(cp, key=lambda x: x[0])
    #     # Draw the controlling point
    #     cv2.circle(middleLineImage, cpDot[1], 3, (255, 0, 255), cv2.FILLED)
    #     print(f"Controlling point: {cpDot[1]}, distance from middle: {cpDot[0]}")

    #     # Display the result
    #     # cv2.imshow("window", image)
    #     cv2.imshow("result", middleLineImage)
    #     cv2.waitKey(0)
