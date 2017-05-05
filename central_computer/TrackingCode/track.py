# Import NumPy Libraries
import numpy as np
from numpy import average as avg
from numpy import subtract as sub

# Import System Libraries
import math
import time
import csv
import sys
from optparse import OptionParser
import socket

# Import OpenCV Libraries
import cv2


# Debug?
DEBUG = False
DISPLAY_TAGS = True

# Colors (BGR codes) for different useful colors
RED = [0, 0, 255]
GREEN = [0, 255, 0]
BLUE = [255, 0, 0]
DARK_BLUE = [255, 51, 51]
YELLOW = [0, 255, 255]
DARK_RED = [0, 0, 170]
MAGENTA = [163, 85, 255]
CUSTOM_COLOR = [163, 85, 255]

# Scaling Pixel Values to CM
CMperPIXEL = 0.1913
FRAMES_PER_SEC = 30
TAG_CENTER_DIST = 8 / CMperPIXEL  # CM adjustment from tag to robot center.

# use the following to run the code profiler
# python -m cProfile camera.py

parser = OptionParser()
parser.add_option('-o',
                  action='store_true',
                  default=False,
                  dest='stdout',
                  help='Sends collected data to stdout.')
parser.add_option('-t',
                  action='store_true',
                  default=False,
                  dest='track',
                  help='Sends collected data to csvFile.')
parser.add_option('-v', '--vid',
                  action='store_true',
                  default=False,
                  dest='vid',
                  help='Save results in a video file.')
parser.add_option('--tracking',
                  action='store',
                  dest='csvfile',
                  default='experiment1.csv',
                  help='give custom name to tracking file (must use .csv)')
parser.add_option('--vidfile',
                  action='store',
                  dest='vidfile',
                  default='experiment1.avi',
                  help='Give custom name to video output file (must use .avi)')
parser.add_option('--notrack',
                  action='store_false',
                  default=False,
                  dest='track',
                  help='Suppress tracking output file.')
parser.add_option('--notags',
                  action='store_false',
                  default=True,
                  dest='displayTags',
                  help='Prevents individual tag windows from spawning.')
options, args = parser.parse_args()

# Create the capture from the webcam.
camera = cv2.VideoCapture(0)
# set the width and height
camera.set(3, 1920)  # Set the width of the capture
camera.set(4, 1080)  # Set the height of the capture
camera.set(5, 30)  # Set the framerate of the capture

# make a call to time.clock to start the clock (future calls to
# time.clock() will report time since this call)
time.clock()
t0 = time.clock()

if options.track:
    f = open(options.csvfile, 'wb')
    writer = csv.writer(f)
    writer.writerow(['TagID', 'center_x', 'center_y',
                     'angle (radians from x axis)', 'time', 'Frame Number'])

if options.vid:
    global OPENSUCCESS
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(options.vidfile, fourcc,
                          FRAMES_PER_SEC, (1920, 1080))


def drawPointPosition(x0, y0, img, color):
    """
    Draws points on specified positions.

    Parameters
    ----------
    x0 : float
        X location for drawn circle.
    y0 : float
        Y location for drawn circle.
    img : obj
        Object frame on which the points will be drawn.
    color : set of RGB values
        Color of the point

    """
    # draw all contours in red, with thickness 2
    cv2.circle(img, (int(x0), int(y0)), 1, color, 2)

##########################################################
# TRACKING ROBOTS SET UP AND FUNCTIONS
##########################################################

# Initialization Constants
VAL = 60
MIN_SIZE = 1000
MAX_SIZE = 2200
MIN_VAL = 100
MAX_VAL = 2000
Robots = {}  # begin with an empty dictionary of robots
MAPPING = {'11': 'l', '30': 29, '40': 22, '03': 27, '22': 28,
           '77': 26}  # Map from the tag number : IP address
mapping = {11: 'l', 30: 29, 40: 22, 3: 27, 22: 28, 77: 26}
sockets = {}

frameNum = 0

# All vectors are assumed to be two dimensional
# The three white squares on the binary tag must be on the robots' left side.

HCELLS = 3  # number of horizontal cells on a tag
VCELLS = 3  # number of vertical cells on a tag


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
    # robot has been updated.

    def __repr__(self):
        center = integerize(
            (self.center[0] * CMperPIXEL, self.center[1] * CMperPIXEL))
        return ("Robot at " + str(center) + " with orientation " +
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


def findAprilTags(threshed, img):
    # Contouring
    contourImage, contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return filter(lambda c: isTag(c, img), contours)


def isTag(c, img):
    # determines if the image is a tag, based on its area and intensity
    return (MIN_SIZE < cv2.contourArea(c) < MAX_SIZE) and \
        (MIN_VAL < averageValue(c) < MAX_VAL) and \
        goodAspectRatio(c)


def goodAspectRatio(c):
    _, (width, height), _ = cv2.minAreaRect(c)
    aspectRatio = max([width / height, height / width])
    return 1 < aspectRatio < 2


def averageValue(img):
    height, width = img.shape[:2]
    val = avg(img.sum(axis=0).sum(axis=0))
    return val / (height * width)


def drawTags(tagList, img):
    """ Draw all contours in red, with thickness 2. """
    cv2.drawContours(img, tagList, -1, DARK_RED, 2)


def drawCorners(bottom, left, top, right, img):
    """ Draw contours in varying colors on an OpenCV image. """
    # draw all contours in red, with thickness 2
    cv2.circle(img, tuple(bottom), 1, BLUE, 2)

    # draw all contours in red, with thickness 2
    cv2.circle(img, tuple(top), 1, BLUE, 2)

    # draw all contours in red, with thickness 2
    cv2.circle(img, tuple(left), 1, BLUE, 2)

    # draw all contours in red, with thickness 2
    cv2.circle(img, tuple(right), 1, BLUE, 2)


# marks each robot with its index and angle on the given image
def drawRobots(img):
    arrow_length = 22
    for index in Robots:
        center = integerize(Robots[index].center)
        angle = Robots[index].orientation
        if index in MAPPING and MAPPING[index] != 'l':
            cv2.circle(img, integerize(center), 2, RED, 4)

        # Robot's name.
        cv2.putText(img, str(index), (center[0] + 28, center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, CUSTOM_COLOR, 2)
        if index in MAPPING and MAPPING[index] != 'l':
            p2 = integerize((center[0] + arrow_length * math.cos(angle),
                             center[1] - arrow_length * math.sin(angle)))
            cv2.line(img, center, p2, (255, 255, 0), 2, 2)


def updateDict(tagList, img, thresh):
    global Robots
    tagViews = []
    for tag in tagList:
        rect = cv2.minAreaRect(tag)
        tagImg = getTagImg(tag, rect, img)
        id_matrix = identify(tagImg)
        # We could not calculate the intensity of the cells, so the image was
        # not a valid tag.
        if id_matrix is None:
            continue

        index = matrixToIndex(id_matrix)
        # The tag did not have three cells on one side which were all light, so
        # the image was not a valid tag.
        if index is None:
            continue

        tagViews.append(tagImg)
        angle = calculateAngle(tag, rect, id_matrix)
        Robots[index] = RobotData(rect[0], angle)
    # Remove any robots from our list that were not updated
    Robots = {key: rob for key, rob in Robots.items() if rob.updated}
    # Reset all robots to their 'non-updated' status for the next iteration.
    for r in Robots.values():
        r.reset()

    return tagViews


def getTagImg(tag, rect, img):
    # extracts the image of the tag from the main image, and rotates it
    # appropriately
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
        theta = math.atan(float('inf'))  # Slope is pi/2.

    height = dist(right, bottom)
    width = dist(right, top)
    if pos_slope:
        width, height = height, width

    fcenter = rect[0][0], rect[0][1]
    return subimage(img, fcenter, theta, width, height)

# Developed from code by user xaedes of Stack Overflow:
# http://stackoverflow.com/questions/11627362/how-to-straighten-a-rotated-rectangle-area-of-an-image-using-opencv-in-python


def subimage(image, center, theta, width, height):

    v_x = (np.cos(theta), np.sin(theta))
    v_y = (-np.sin(theta), np.cos(theta))
    s_x = center[0] - v_x[0] * (width / 2) - v_y[0] * (height / 2)
    s_y = center[1] - v_x[1] * (width / 2) - v_y[1] * (height / 2)

    new_mapping = np.array([[v_x[0], v_y[0], s_x],
                            [v_x[1], v_y[1], s_y]])

    return cv2.warpAffine(image, new_mapping, (width, height),
                          flags=cv2.WARP_INVERSE_MAP,
                          borderMode=cv2.BORDER_REPLICATE)


def identify(img):
    x_buff = 6  # pixels of buffer zone in x
    y_buff = 3  # pixels of buffer zone in y
    matrix = np.zeros((VCELLS, HCELLS), dtype=bool)
    threshed = threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 132)
    h, w, _ = np.shape(img)
    x, y = 1, 1
    dx = int((w - 2 * x_buff) / float(HCELLS))
    dy = int((h - 2 * y_buff) / float(VCELLS))
    for i in range(HCELLS):
        for j in range(VCELLS):
            # Because we're interested in the white squares now
            white = not isBlack(threshed,
                                (x + x_buff + i * dx, y + y_buff + j * dy),
                                (x + x_buff + (i + 1) * dx,
                                 y + y_buff + (j + 1) * dy), dx * dy, img)
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
    # Dark squares will have an intensity below this percentage.
    DARKNESS_THRESHOLD = 0.3
    intensity = 0
    p1, p2 = integerize(p1), integerize(p2)
    for x in range(p1[0], p2[0]):
        for y in range(p1[1], p2[1]):
            intensity += bool(img[y, x])
            if x in (p1[0], p2[0] - 1) or y in (p1[1], p2[1] - 1):
                defacing[y, x] = RED

    # This means that we are picking up some edge motion.
    if area == 0:
        return None

    filled = (intensity / float((p2[1] - p1[1]) *
                                (p2[0] - p1[0]))) < DARKNESS_THRESHOLD
    return filled

# calculates the cartesian distance between two points


def dist(p1, p2):
    return np.linalg.norm(sub(p1, p2))


def calculateAngle(tag, rect, id_matrix):
    bottom, left, top, right = cv2.boxPoints(rect)
    if dist(left, top) < dist(left, bottom):
        if left[0] == bottom[0]:
            theta = math.atan(-float('inf'))  # Avoid division by zero.
        else:
            theta = math.atan2((bottom[1] - left[1]), (left[0] - bottom[0]))
        theta -= math.pi / 2

    else:
        if right[0] == bottom[0]:
            theta = math.atan(-float('inf'))  # Avoid division by zero.
        else:
            theta = math.atan2((bottom[1] - left[1]), (left[0] - bottom[0]))

    # Top is light.
    if id_matrix[0, 0] and id_matrix[1, 0] and id_matrix[2, 0]:
        return theta
    # Bottom is light.
    elif id_matrix[0, 2] and id_matrix[1, 2] and id_matrix[2, 2]:
        return theta + math.pi

    # No else case because any such matricies would already be filtered out by
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


def fixRobotData(robotID):
    global Robots
    if robotID in MAPPING and MAPPING[robotID] != 'l':
        center = Robots[robotID].center
        angle = Robots[robotID].orientation
        trueCenter = (center[0], center[1])
        Robots[robotID].center = trueCenter


def integerize(tup):
    return tuple([int(x) for x in tup])


def writeData(frameNum):
    for robot_index in Robots:
        robot = Robots[robot_index]
        centerxCentimeters = robot.center[0] * CMperPIXEL
        centeryCentimeters = robot.center[1] * CMperPIXEL
        if options.stdout:
            if not sys.excepthook:
                exit(0)
            try:
                print(robot_index, centerxCentimeters, centeryCentimeters,
                      robot.orientation, time.clock(), frameNum)

            except IOError:
                exit(0)

        if options.track:
            writer.writerow([robot_index, centerxCentimeters,
                             centeryCentimeters, robot.orientation,
                             time.clock(), frameNum])


def tagToRobot(ID):
    if int(ID) in mapping:
        return mapping[int(ID)]
    else:
        return None


def sendCommand(robots):
    global sockets

    bots = [list(x) for x in robots]
    bots = filter(lambda b: b[0] in mapping, bots)

    bots = sorted(bots, key=lambda bot: bot[4])

    for robotInd in range(len(bots)):
        tup = integerize(
            (bots[robotInd][1], bots[robotInd][2], bots[robotInd][3]))
        cmd = 'M!'.format(*tup)
        robotNumber = tagToRobot(int(bots[robotInd][0]))
        if robotNumber is None:
            continue

        if robotNumber in sockets:
            safesend(cmd, sockets[robotNumber])
        else:
            dest = '192.168.119.{0}'.format(robotNumber)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.settimeout(4)
                s.connect((dest, 12345))
                s.settimeout(None)

            except socket.error:
                print("Could not connect to", dest)
                exit(0)

            safesend(cmd, s)
            sockets[robotNumber] = s
    return


def sendStopCommand(robots):
    global sockets

    bots = [list(x) for x in robots]
    bots = filter(lambda b: b[0] in mapping, bots)
    bots = sorted(bots, key=lambda bot: bot[4])

    for robotInd in range(len(bots)):
        tup = integerize(
            (bots[robotInd][1], bots[robotInd][2], bots[robotInd][3]))
        cmd = 'S!'.format(*tup)
        robotNumber = tagToRobot(int(bots[robotInd][0]))
        if robotNumber is None:
            continue

        if robotNumber in sockets:
            safesend(cmd, sockets[robotNumber])
        else:
            dest = '192.168.119.{0}'.format(robotNumber)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.settimeout(4)
                s.connect((dest, 12345))
                s.settimeout(None)

            except socket.error:
                print("Could not connect to", dest)
                exit(0)

            safesend(cmd, s)
            sockets[robotNumber] = s
    return


def safesend(msg, socket):
    totalSent = 0
    while totalSent < len(msg):
        sent = socket.send(msg[totalSent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")

        totalSent = totalSent + sent

    print("Sent", msg)


def closeAll():
    for sock in sockets.values():
        sock.close()


#########################################################
# MAIN
#########################################################
while True:
    # Determine Time Step
    deltaT = time.clock() - t0
    t0 += deltaT

    # Capture Frame
    ret, image = camera.read()

    # Operations on the frame
    thresh = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(thresh, (3, 3), 0)

    cv2.threshold(gray, VAL, 255, cv2.THRESH_BINARY, thresh)

    # Find Tags
    tagList = findAprilTags(gray, image)
    tagViews = updateDict(tagList, image, thresh)

    # Translate Tag Centers to Robot Centers.
    for robot_index in Robots:
        fixRobotData(robot_index)

    robotList = []

    for robot_index in Robots:
        if robot_index not in MAPPING:
            continue

        robot = Robots[robot_index]
        robotList.append([int(robot_index), robot.center[0] * CMperPIXEL,
                          robot.center[1] * CMperPIXEL,
                          robot.orientation * 180 / math.pi + 180,
                          time.clock(), frameNum])

    # Draw Tags.
    drawTags(tagList, image)
    drawRobots(image)

    # Resize the image to fit screen.
    resizedImage = cv2.resize(image, (1280, 720))

    # Display the images.
    cv2.imshow('Tracking!', resizedImage)
    if frameNum > 50:
        if frameNum % 10 == 0:
            for robot_index in Robots:
                robot = Robots[robot_index]
                sendCommand(robotList)

    if options.track or options.stdout:
        writeData(frameNum)

    if options.vid:
        cv2.putText(image, '{:.2f}'.format(time.clock()),
                    (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0))
        out.write(image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if time.clock() >= 30:
        break

    frameNum += 1

# Clean Up
if options.track:
    f.close()

for robot_index in Robots:
    sendStopCommand(robotList)
closeAll()
camera.release()
cv2.destroyAllWindows()
