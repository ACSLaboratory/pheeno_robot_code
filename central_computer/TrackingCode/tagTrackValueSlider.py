"""
Author: Sean Wilson based heavily upon Michael Sheely's tracking code
from UCLA REU (camera.py).
"""
# Import NumPy Libraries
import numpy as np
from numpy import average as avg
from numpy import subtract as sub

# Import System Libraries
import math

# Import OpenCV Libraries
import cv2


def update(x):
    """ Get track bar position and update its value. """
    global VAL
    global MIN_SIZE
    global MAX_SIZE
    global MIN_VAL
    global MAX_VAL
    global DARKNESS_THRESHOLD
    VAL = cv2.getTrackbarPos('VAL', 'trackFrame')
    MIN_SIZE = cv2.getTrackbarPos('MIN_SIZE', 'trackFrame')
    MAX_SIZE = cv2.getTrackbarPos('MAX_SIZE', 'trackFrame')
    MIN_VAL = cv2.getTrackbarPos('MIN_VAL', 'trackFrame')
    MAX_VAL = cv2.getTrackbarPos('MAX_VAL', 'trackFrame')
    DARKNESS_THRESHOLD = cv2.getTrackbarPos('DARKNESS_THRESHOLD', 'trackFrame')

""" Create window and track bars. """
cv2.namedWindow('trackFrame', cv2.WINDOW_NORMAL)
cv2.createTrackbar('VAL', 'trackFrame', VAL, 200, update)
cv2.createTrackbar('MIN_SIZE', 'trackFrame', MIN_SIZE, 2000, update)
cv2.createTrackbar('MAX_SIZE', 'trackFrame', MAX_SIZE, 4000, update)
cv2.createTrackbar('MIN_VAL', 'trackFrame', MIN_VAL, 200, update)
cv2.createTrackbar('MAX_VAL', 'trackFrame', MAX_VAL, 2000, update)
cv2.createTrackbar('DARKNESS_THRESHOLD', 'trackFrame',
                   DARKNESS_THRESHOLD, 100, update)


class RobotData(object):
    """
    Robot Data class

    Attributes
    ----------
    center : Set of integers
        The center position of the robot.
    orientation : float
        Something
    updated : bool
        The state of the classes attributes. If false, it will allow for
        future value updates.

    Methods
    -------
    update(center, angle)
        Updates all the attributes of the class.
    reset()
        Resets the updated boolean attribute for future value updates.

    """
    def __init__(self, center, orientation):
        self.center = center
        self.orientation = orientation
        self.updated = True

    # center is an (x,y) tuple, orientation is an angle in degrees measured
    # from the positive x axis, frame is a number which designates which frame
    # the robot is in, and updated is a boolean which tells if that particular
    # robot has been updated

    def __repr__(self):
        new_center = integerize(
            (self.center[0] * CMperPIXEL, self.center[1] * CMperPIXEL))
        return ("Robot at " + str(new_center) + " with orientation " +
                str(self.orientation) + ".")

    def update(self, updated_center, updated_angle):
        self.center = updated_center
        self.orientation = updated_angle
        self.updated = True

    def reset(self):
        self.updated = False


def threshold(src, value=100):
    ret, thresh = cv2.threshold(src, VAL, 255, cv2.THRESH_BINARY)
    return thresh


def findAprilTags(thresh, img):
    # Contouring
    contourImage, contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return filter(lambda c: isTag(c, img), contours)


def isTag(c, img):
    """ Determines if the image is a tag, based on its area and intensity. """
    return (MIN_SIZE < cv2.contourArea(c) < MAX_SIZE) and \
        (MIN_VAL < averageValue(c) < MAX_VAL) and \
        goodAspectRatio(c)


def goodAspectRatio(c):
    _, (width, height), _ = cv2.minAreaRect(c)
    aspect_ratio = max([width / height, height / width])
    return 1 < aspect_ratio < 2


def averageValue(img):
    height, width = img.shape[:2]
    val = avg(img.sum(axis=0).sum(axis=0))
    return val / (height * width)


def drawTags(tagList, img):
    """ Draw all contours in red, with thickness 2 """
    cv2.drawContours(img, tagList, -1, DARK_RED, 2)


def drawCorners(bottom, left, top, right, img):
    """ Draw contours in varying colors on an OpenCV image. """
    # Draw all contours in green, with thickness 2.
    cv2.circle(img, tuple(bottom), 1, GREEN, 2)

    # Draw all contours in blue, with thickness 2.
    cv2.circle(img, tuple(top), 1, BLUE, 2)

    # Draw all contours in dark red, with thickness 2.
    cv2.circle(img, tuple(left), 1, DARK_RED, 2)

    # Draw all contours in a custom color, with thickness 2.
    cv2.circle(img, tuple(right), 1, CUSTOM_COLOR, 2)


def drawRobots(img):
    """ Marks each robot with its index and angle on an OpenCV image. """
    arrow_length = 22
    for index in Robots:
        new_center = integerize(Robots[index].center)
        angle = Robots[index].orientation
        if index in MAPPING and MAPPING[index] != 'l':
            cv2.circle(img, integerize(new_center), 2, RED, 4)

        # Robot's name
        cv2.putText(img, str(index), (new_center[0] + 28,
                                      new_center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    .7, CUSTOM_COLOR, 2)
        if index in MAPPING and MAPPING[index] != 'l':
            p2 = integerize((new_center[0] + arrow_length * math.cos(angle),
                             new_center[1] - arrow_length * math.sin(angle)))
            cv2.line(img, new_center, p2, (255, 255, 0), 2, 2)


def updateDict(tag_list, img, thresh):
    global Robots
    tag_views = []
    for tag in tag_list:
        rect = cv2.minAreaRect(tag)
        tag_img = getTagImg(tag, rect, img)
        id_matrix = identify(tag_img)
        # Get's the commands from the message to see if they are for
        # the gripper.
        if id_matrix is None:
            continue

        index = matrixToIndex(id_matrix)
        if index is None:
            # Get's the commands from the message to see if they are for
            # the gripper.
            continue

        tag_views.append(tag_img)
        angle = calculateAngle(tag, rect, id_matrix)
        Robots[index] = RobotData(rect[0], angle)

    # remove any robots from our list that were not updated
    Robots = {key: rob for key, rob in Robots.items() if rob.updated}

    # Get's the commands from the message to see if they are for the gripper.
    for r in Robots.values():
        r.reset()

    return tag_views


def getTagImg(tag, rect, img):
    """
    Extracts the image of the tag from the main image, and rotates it
    appropriately.
    """
    bottom, left, top, right = cv2.boxPoints(rect)
    # drawCorners(bottom, left, top, right, imageTrack)
    try:
        if dist(left, top) < dist(left, bottom):
            pos_slope = False
            theta = math.atan((left[1] - bottom[1]) / (left[0] - bottom[0]))
        else:
            pos_slope = True
            theta = math.atan((right[1] - bottom[1]) / (right[0] - bottom[0]))

    except ZeroDivisionError:
        theta = math.atan(float('inf'))  # slope is pi/2

    height = dist(right, bottom)
    width = dist(right, top)
    if pos_slope:
        width, height = height, width

    f_center = rect[0][0], rect[0][1]
    return subimage(img, f_center, theta, width, height)

# Developed from code by user xaedes of stack overflow
# http://stackoverflow.com/questions/11627362/how-to-straighten-a-rotated-rectangle-area-of-an-image-using-opencv-in-python


def subimage(image, center, theta, width, height):
    v_x = (np.cos(theta), np.sin(theta))
    v_y = (-np.sin(theta), np.cos(theta))
    s_x = center[0] - v_x[0] * (width / 2) - v_y[0] * (height / 2)
    s_y = center[1] - v_x[1] * (width / 2) - v_y[1] * (height / 2)

    mapping = np.array([[v_x[0], v_y[0], s_x],
                        [v_x[1], v_y[1], s_y]])

    return cv2.warpAffine(image, mapping, (width, height),
                          flags=cv2.WARP_INVERSE_MAP,
                          borderMode=cv2.BORDER_REPLICATE)


def identify(img):
    XBUF = 6  # pixels of buffer zone
    YBUF = 3
    matrix = np.zeros((VCELLS, HCELLS), dtype=bool)
    threshed = threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 132)
    h, w, _ = np.shape(img)
    x, y = 1, 1
    dx = int((w - 2 * XBUF) / float(HCELLS))
    dy = int((h - 2 * YBUF) / float(VCELLS))
    for i in range(HCELLS):
        for j in range(VCELLS):
            # Because we're interested in the white squares now
            white = not isBlack(threshed,
                                (x + XBUF + i * dx, y + YBUF + j * dy),
                                (x + XBUF + (i + 1) * dx,
                                 y + YBUF + (j + 1) * dy), dx * dy, img)
            if white is not None:
                matrix[j, i] = white
            else:
                return None

    return matrix


def largestContour(contour_list):
    contour = None
    size = 0
    for current in contour_list:
        current_area = cv2.contourArea(current)
        if current_area > size:
            contour = current
            size = current_area

    return contour


def isBlack(img, p1, p2, area, defacing):
    # dark squares will have an intensity below this percentage
    DT = DARKNESS_THRESHOLD / 100.0
    intensity = 0
    p1, p2 = integerize(p1), integerize(p2)
    for x in range(p1[0], p2[0]):
        for y in range(p1[1], p2[1]):
            intensity += bool(img[y, x])
            if x in (p1[0], p2[0] - 1) or y in (p1[1], p2[1] - 1):
                defacing[y, x] = RED

    if area == 0:
        return None  # this means that we are picking up some edge motion

    filled = (intensity / float((p2[1] - p1[1]) * (p2[0] - p1[0]))) < DT
    return filled


def dist(p1, p2):
    """ Calculates the cartesian distance between two numpy array points. """
    return np.linalg.norm(sub(p1, p2))


def calculateAngle(tag, rect, id_matrix):
    bottom, left, top, right = cv2.boxPoints(rect)
    drawCorners(bottom, left, top, right, imageTrack)
    if dist(left, top) < dist(left, bottom):
        if left[0] == bottom[0]:
            theta = math.atan(-float('inf'))  # avoid division by zero
        else:
            theta = math.atan2((bottom[1] - left[1]), (left[0] - bottom[0]))

        theta -= math.pi / 2

    else:
        if right[0] == bottom[0]:
            theta = math.atan(-float('inf'))  # avoid division by zero
        else:
            theta = math.atan2((bottom[1] - left[1]), (left[0] - bottom[0]))

    # Top is light
    if id_matrix[0, 0] and id_matrix[1, 0] and id_matrix[2, 0]:
        return theta
    # Bottom is light
    elif id_matrix[0, 2] and id_matrix[1, 2] and id_matrix[2, 2]:
        return theta + math.pi

    # no else case because any such matricies would already be filtered out by
    # matrixToIndex (returns None)


def binaryDigitsToDecimalString(L):
    return str(int(''.join([str(int(x)) for x in L]), 2))


def matrixToIndex(matrix):
    if np.all(matrix[:, 0]):
        index = binaryDigitsToDecimalString(
            matrix[:, 2]) + binaryDigitsToDecimalString(matrix[:, 1])

    elif np.all(matrix[:, 2]):
        index = binaryDigitsToDecimalString(
            matrix[:, 0][::-1]) + \
            binaryDigitsToDecimalString(matrix[:, 1][::-1])

    else:
        index = None

    return index


def fixRobotData(robot_id, tag_center_dist):
    global Robots
    if robot_id in MAPPING and MAPPING[robot_id] != 'l':
        center = Robots[robot_id].center
        angle = Robots[robot_id].orientation
        true_center = (center[0] + math.cos(angle) * tag_center_dist,
                       center[1] - math.sin(angle) * tag_center_dist)
        Robots[robot_id].center = true_center


def integerize(point):
    """ Creates a set of integer values. """
    return (int(point[0]), int(point[1]))


""" Initialization Constants """
VAL = 90
MIN_SIZE = 1500
MAX_SIZE = 2200
MIN_VAL = 100
MAX_VAL = 1200
DARKNESS_THRESHOLD = 40
CUSTOM_COLOR = [163, 85, 255]

# BGR codes for different useful colors
RED = [0, 0, 255]
GREEN = [0, 255, 0]
BLUE = [255, 0, 0]
DARK_BLUE = [255, 51, 51]
YELLOW = [0, 255, 255]
DARK_RED = [0, 0, 170]
MAGENTA = [163, 85, 255]

Robots = {}  # begin with an empty dictionary of robots
MAPPING = {'11': 'l', '33': 33}  # Map from the tag number : IP address
CMperPIXEL = 0.143

DISPLAY_TAGS = True

# All vectors are assumed to be two dimensional
# The three dark squares must be on the robots' left side.

HCELLS = 3  # Number of horizontal cells on a tag
VCELLS = 3  # Number of vertical cells on a tag

""" Initialize Camera """
camera = cv2.VideoCapture(0)  # Create the capture from the webcam.
camera.set(3, 1920)  # Set the width of the capture.
camera.set(4, 1080)  # Set the height of the capture.
camera.set(5, 60)    # Set the framerate of the capture.

while True:
    # Capture Frame
    ret, image = camera.read()
    ret2, imageTrack = camera.read()

    # Operations on the frame
    thresh = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(thresh, (3, 3), 0)

    # Thresholding
    cv2.threshold(gray, VAL, 255, cv2.THRESH_BINARY, thresh)
    resizedThresh = cv2.resize(thresh, (1280, 720))
    cv2.imshow('Threshold', resizedThresh)

    # Find Tags
    tagList = findAprilTags(gray, imageTrack)
    tagViews = updateDict(tagList, imageTrack, thresh)

    # Draw Tags
    drawTags(tagList, imageTrack)
    drawRobots(imageTrack)

    # Resize the image to fit screen.
    resizedImage = cv2.resize(image, (1280, 720))
    resizedImageTrack = cv2.resize(imageTrack, (1280, 720))

    # Display the images
    cv2.imshow('Original Image', resizedImage)
    cv2.imshow('Tag Recognition', resizedImageTrack)

    print(Robots.values())

    if DISPLAY_TAGS:
        for i in range(len(tagViews)):
            avgValue = averageValue(tagViews[i])
            center = (10, 10)
            cv2.imshow(str(i), tagViews[i])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean Up
camera.release()
cv2.destroyAllWindows()
