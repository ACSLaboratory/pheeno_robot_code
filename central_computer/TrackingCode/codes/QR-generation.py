"""
Author: Michael Sheely
June 24, 2015
this is a command line python program
open a command window and type
python QR-generation.py code outputFile
Where code is a two digit number with first digit 0-6, second digit 0-7
If you wish to generate multiple codes in a single output file, create
a file of newline codes and then type
python QR-generation.py -f fileContainingCodes
In this case, the script will automatically send the output to codes.jpg
in the same directory
"""

import cv2
import numpy as np
from optparse import OptionParser

# Define some constants for the QR codes
CODES_TO_CREATE = 3  # by default, we will produce three codes
CODES_PER_LINE = 3
XBUF = 20 * 2
YBUF = 24 * 2
WIDTH = 108 * 2
HEIGHT = 183 * 2
OFFSET = 30 * 1.5
ROWS = 3
COLS = 3
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


def main():
    global WHITE, BLACK
    usage = "usage: QR-generation.py [options]"
    parser = OptionParser(usage=usage)
    parser.add_option('--invert',
                      action='store_true',
                      default=False,
                      dest='invert',
                      help='invert colors of output image')
    parser.add_option('-f',
                      action='store',
                      default=False,
                      dest='identifiers',
                      help='provide a newline separated file listing codes to create')
    options, args = parser.parse_args()
    if options.identifiers:
        pics = createQRfromFile(options.identifiers, options.invert)
        if options.invert:
            pics = invertPics(pics)

        for page in range(len(pics)):
            cv2.imwrite('codes' + str(page) + '.jpg', pics[page])

        exit(0)
    codeIdentifier = raw_input('Input code identifier to create.\n')
    c1, c2 = extractCode(codeIdentifier)
    if c1 is None:
        exit(0)

    pic = createQR(c1, c2)
    if options.invert:
        pic = invertPics([pic])[0]

    cv2.imwrite('codes0.jpg', pic)


def extractCode(codeIdentifier):
    # assumes length of codeIdentifier has been confirmed to be 2
    c1, c2 = integerize(codeIdentifier)
    if c1 in range(7) and c2 in range(8):
        return c1, c2
    else:
        print("Error on: " + str(c1) + str(c2))
        print("codeIdentifier should be a two digit number with first digit " +
              "0-6, second digit 0-7")
        return (None, None)


def createQR(c1, c2):
    BUFF = 1
    picture = np.ones((HEIGHT, WIDTH, 3), np.uint8) * 255
    # express bitwise and of 111 and c1 as a three digit binary number
    # left-filled with zeros
    topRow = '{0:03b}'.format(7 & c1)
    midRow = '{0:03b}'.format(7 & c2)
    a, b, c = integerize(topRow)
    d, e, f = integerize(midRow)
    matrix = np.array([[a, b, c], [d, e, f], [1, 1, 1]])
    dy = (HEIGHT - 2 * YBUF) / float(ROWS)
    dx = (WIDTH - 2 * XBUF) / float(COLS)
    for y in range(ROWS):
        for x in range(COLS):
            y0 = int(YBUF + y * dy)
            y1 = int(YBUF + (y + 1) * dy)
            x0 = int(XBUF + x * dx)
            x1 = int(XBUF + (x + 1) * dx)
            picture[y0:y1, x0:x1] = [
                255 * int(not matrix[y, x]) for i in range(3)]
    picture[0:BUFF, :] = BLACK
    picture[HEIGHT - BUFF:HEIGHT, :] = BLACK
    picture[:, 0:BUFF] = BLACK
    picture[:, WIDTH - BUFF:WIDTH] = BLACK
    return picture


def createQRfromFile(filename, inverted=False):
    f = open(filename)
    counter = 0  # keeps track of how many codes we've created
    pictures = []
    for line in f:
        # Get's the commands from the message to see if they are for the
        # gripper.
        if (counter % CODES_PER_LINE) == 0:
            pictures.append(np.zeros((HEIGHT, OFFSET, 3), np.uint8))

        # remove the "newline" character from each line in input file
        line = line[:-1]
        if len(line) != 2:
            print("Each codeIdentifier in %s should be a two digit number " +
                  " with first digit 0-6, second digit 0-7" % filename)
            continue

        c1, c2 = extractCode(line)
        pictures.append(createQR(c1, c2))
        # add the black bar after each photo
        pictures.append(np.zeros((HEIGHT, OFFSET, 3), np.uint8))
        counter += 1

    pages = []

    while pictures:  # there are more codes to be read
        # grab at most the first three codes (and the four black bars in
        # between)
        first3Codes = safeSlice(pictures, 0, CODES_PER_LINE * 2 + 1)
        # at most the next three codes (and the four black bars in between)
        second3Codes = safeSlice(
            pictures, CODES_PER_LINE * 2 + 1, CODES_PER_LINE * 4 + 2)
        # Get's the commands from the message to see if they are for the
        # gripper.
        if second3Codes is []:
            # (concatenated together)
            pages.append(np.concatenate(first3Codes, axis=1))
            return pages

        row0 = np.concatenate(first3Codes, axis=1)
        row1 = np.concatenate(second3Codes, axis=1)
        # if there were not a full six tags, row1 is shorter than row0
        deficiency = np.shape(row0)[1] - np.shape(row1)[1]
        if deficiency > 0:
            if inverted:
                # so add whitespace to row1 so we can concatenate them
                # vertically
                row1 = np.concatenate(
                    (row1, np.zeros((HEIGHT, deficiency, 3))), axis=1)
            else:
                # so add whitespace to row1 so we can concatenate them
                # vertically
                row1 = np.concatenate(
                    (row1, 255 * np.ones((HEIGHT, deficiency, 3))), axis=1)

        # add the new page to the list of pages
        pages.append(np.concatenate((row0, row1), axis=0))
        # and remove the codes (and black bars) that we just saved onto a page
        pictures = safeSlice(pictures, CODES_PER_LINE * 4 + 2, 'END')
    return pages


def safeSlice(L, imin, imax):
    if imax == 'END' or imax > len(L):
        return L[imin:]
    else:
        return L[imin:imax]


def invertPics(piclist):
    result = []
    for pic in piclist:
        result.append((255 - pic))
    return result


def integerize(L):
    return [int(x) for x in L]

if __name__ == '__main__':
    main()
