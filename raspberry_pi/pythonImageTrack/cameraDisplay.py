#!/usr/bin/python

import picamera
import picamera.array
import cv2
import time

time.clock()
t0 = time.clock()
minTime = 10000
maxTime = 0
meanTime = 0

count = 1

""" IMPORT CONFIG FILE WITH HSV TRACKING VALUES! """


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


""" CAMERA ROUTINE! """
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (320, 240)

        while True:
            camera.capture(stream, 'bgr', use_video_port=True)
            # stream.array contains the image array in bgr order!
            # gray = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
            # HSV = cv2.cvtColor(stream.array, cv2.COLOR_BGR2HSV)
            # cv2.imshow('Video!', stream.array)
            # cv2.imshow('GRAY!', gray)
            # cv2.imshow('HSV!', HSV)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the array for next capture
            stream.seek(0)
            stream.truncate()

            if count > 10:
                (minTime, meanTime, maxTime) = updateTimeData(
                    t0, maxTime, meanTime, minTime, count - 10)
            print(minTime, meanTime, maxTime)
            t0 = time.clock()
            count = count + 1

        cv2.destroyAllWindows()
