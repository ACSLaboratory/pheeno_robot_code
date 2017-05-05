import cv2
import numpy as np

"""
The following code creates a window display for the overhead camera,
NOT the onboard camera.
"""

if __name__ == '__main__':
    camera1 = cv2.VideoCapture(0)
    # set camera parameters
    camera1.set(3, 1920)  # Set the width of the capture
    camera1.set(4, 1080)  # Set the height of the capture
    camera1.set(5, 60)    # Set the framerate of the capture

    #########################################################
    #  MAIN
    #########################################################
    while True:
        # Capture Frame
        ret1, image1 = camera1.read()

        # Display the images
        cv2.imshow('Microsoft Life Cam!', image1)

        width, height, channels = image1.shape

        print(width, height, channels)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera1.release()
    cv2.destroyAllWindows()
