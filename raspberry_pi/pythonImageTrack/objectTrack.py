#!/usr/bin/python

import picamera
import picamera.array
import cv2
import numpy as np
import time
import sys

# --- Initial Variables --- #
debug = False  # Will display video.
thresh = dict()
CM = []
maxObjects = 3  # Number of objects of each color to track max.
# Will not track if this is exceded (will be thought as a noisy image).
minObjectArea = 700  # Minimum Object Area Accepted.
FRAME_HEIGHT = 240
FRAME_WIDTH = 320
rg = 11  # Red Gain for White Balance
bg = 22  # Blue Gain for White Balance

time.clock()
t0 = time.clock()
minTime = 10000
maxTime = 0
meanTime = 0

count = 1


def fromFile(filename):
    """
    Opens config fine and places HSV tracking values in an array.

    Parameters
    ----------
    filename : str
        File path for HSV tracking values.

    Returns
    -------
    results : array
        Array containing HSV tracking values.

    """
    f = open(filename, "r")
    results = [x.rstrip('\n').split() for x in open(filename).readlines()]
    f.close()
    return results


def updateTimeData(oldTime, oldMaxTime, oldMeanTime, oldMinTime, frameNum):
    """
    Updates relevant time data. These are compared to old time data and then
    returned by the function.

    Parameters
    ----------
    oldTime : float
        The previous time value.
    oldMaxTime : float
        Previous maximum time value.
    oldMeanTime : float
        Previous mean time value.
    oldMinTime : float
        Previous minimum time value.
    frameNum : int
        The current frame number.

    Returns
    -------
    minTime : float
        New minimum time value.
    meanTime : float
        New mean time value.
    maxTime : float
        New max time value.

    """
    timeTemp = time.clock() - oldTime
    if timeTemp < oldMinTime:
        minTime = timeTemp
    else:
        minTime = oldMinTime

    if timeTemp > oldMaxTime:
        maxTime = timeTemp
    else:
        maxTime = oldMaxTime

    meanTime = float(frameNum - 1) / float(frameNum) * \
        oldMeanTime + 1 / float(frameNum) * timeTemp
    return (minTime, meanTime, maxTime)


def morphOps(binaryMatrix):
    kernel = np.ones((15, 15), np.uint8)
    fix = cv2.morphologyEx(binaryMatrix, cv2.MORPH_CLOSE, kernel)
    return fix


def drawCrossHair(frame, x, y, color):
    """
    Creates a visible crosshair on an object in a specific frame.

    Parameters
    ----------
    frame : obj
        The frame the object is located.
    x : float
        The x position of the object.
    y : float
        The y position of the object.
    color : set of RGB values
        The color of the object.

    """
    crossHairRadius = 10
    cv2.circle(frame, (x, y), crossHairRadius, (0, 255, 0), 2)
    cv2.putText(frame, color, (x - 30, y - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    if (y - crossHairRadius) > 0:
        cv2.line(frame, (x, y), (x, y - crossHairRadius), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (x, 0), (0, 255, 0), 2)

    if (y + crossHairRadius) < FRAME_HEIGHT:
        cv2.line(frame, (x, y), (x, y + crossHairRadius), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (x, FRAME_HEIGHT), (0, 255, 0), 2)

    if (x - crossHairRadius) > 0:
        cv2.line(frame, (x, y), (x - crossHairRadius, y), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (0, y), (0, 255, 0), 2)

    if (x + crossHairRadius) < FRAME_WIDTH:
        cv2.line(frame, (x, y), (x + crossHairRadius, y), (0, 255, 0), 2)
    else:
        cv2.line(frame, (x, y), (FRAME_WIDTH, y), (0, 255, 0), 2)


def findObjects(binaryMatrix, color):
    """
    Locate objects of a specific color and return the positions (and color)
    of the located objects.

    Parameters
    ----------
    binaryMatrix : val
        Something
    color : set of RGB values
        The color of the object.

    Returns
    -------
    output : array of arrays
        Each entry holds an array containing the x, y position and color of
        the object found.

    """
    objectFound = False
    output = []
    binaryMatrix = morphOps(binaryMatrix)
    contours, hierarchy = cv2.findContours(
        binaryMatrix, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cont = sorted(contours, key=cv2.contourArea, reverse=True)[:maxObjects]
    if hierarchy is not None:
        for i in range(0, len(cont)):
            M = cv2.moments(cont[i])
            if M['m00'] > minObjectArea:
                if debug:
                    cv2.drawContours(stream.array, cont[i], -1,
                                     (255, 0, 0), 3)
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])

                if output == []:
                    output = [[x, y, color]]
                else:
                    output.append([x, y, color])

    return output


# IMPORT CONFIG FILE WITH HSV TRACKING VALUES!
HSV_VALS = fromFile("/home/pi/CrowVisit/colorConfig.txt")

""" CAMERA ROUTINE! """
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)
        camera.vflip = True
        camera.awb_mode = 'off'
        camera.awb_gains = (float(rg / 10), float(bg / 10))

        while True:
            try:
                camera.capture(stream, 'bgr', use_video_port=True)
                # stream.array contains the image array in bgr order!
                # Do color conversions/blurs
                blur = cv2.GaussianBlur(stream.array, (5, 5), 0)
                HSV = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

                # Threshold the image
                for i in range(0, int(np.array(HSV_VALS).shape[0])):
                    thresh[i] = cv2.inRange(HSV,
                                            np.array((int(HSV_VALS[i][1]),
                                                      int(HSV_VALS[i][3]),
                                                      int(HSV_VALS[i][5])),
                                                     np.uint8),
                                            np.array((int(HSV_VALS[i][2]),
                                                      int(HSV_VALS[i][4]),
                                                      int(HSV_VALS[i][6])),
                                                     np.uint8))
                    thresh[i] = morphOps(thresh[i])
                    centers = findObjects(thresh[i], HSV_VALS[i][0])
                    if (centers != []):
                        if CM == []:
                            CM = centers
                        else:
                            CM.extend(centers)

                mask = thresh[0]
                for k in range(1, int(np.array(HSV_VALS).shape[0])):
                    mask = mask + thresh[k]

                if CM == []:
                    print("No Object Found!")
                else:
                    print(CM)

                if debug:
                    for i in range(0, len(CM)):
                        drawCrossHair(stream.array, CM[i][
                                      0], CM[i][1], CM[i][2])

                    cv2.imshow('Video!', stream.array)
                    # cv2.imshow('HSV!', HSV)
                    # cv2.imshow('Threshold!',thresh[0])

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                if count > 10:
                    res = cv2.bitwise_and(
                        stream.array, stream.array, mask=mask)
                    for i in range(0, len(CM)):
                        drawCrossHair(stream.array, CM[i][0],
                                      CM[i][1], CM[i][2])
                    (minTime, meanTime, maxTime) = updateTimeData(
                        t0, maxTime, meanTime, minTime, count - 10)

                if count == 200:
                    print(inTime, meanTime, maxTime)

                t0 = time.clock()
                count += 1
                # reset the array for next capture
                stream.seek(0)
                stream.truncate()
                CM = []
                mask = []

            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                sys.exit("Successfully Exited")

        cv2.destroyAllWindows()
