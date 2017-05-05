# System Imports
import subprocess
import math
import time
import sys
import os


class Gripper:
    """
    Gripper control.

    The class allows for interaction with the Pheeno's gripper module.

    Attributes
    ----------
    udg : int
        Desired up-down gripper position.
    rotG : int
        Desired roll gripper position.
    yG : int
        Desired yaw gripper position.
    cG : int
        Desired claw gripper position.
    pi : float
        A simple representation of the mathematical value pi.
    delayRatio : float
        A delay to prevent too fast of movement on the servo.
    udPos : int
        Current up-down gripper position.
    rotPos : int
        Current roll gripper position.
    yPos : int
        Current yaw gripper position.
    cPos : int
        Current claw gripper position.

    Methods
    -------
    gripperMove(servo, angle)
        Moves Pheeno's gripper to the specified angle. Allows a user to
        specify which servo to move.
    manualInput()
        Manual control of servos using a text-interface approach.
    allOff()
        Turns off the Pheeno's servos on the gripper.

    """

    def __init__(self):
        self.udG = 0
        self.rotG = 1
        self.yG = 2
        self.cG = 3
        self.pi = 3.14
        # 0.12 ms per 60 deg of rotation delay needed.
        self.delayRatio = 0.12 / 60

        self.udPos = -360
        self.rotPos = -360
        self.yPos = -360
        self.cPos = -360

        if not os.geteuid() == 0:  # Make sure the script is run as sudo.
            sys.exit("\nOnly root can run this script\n")

        subprocess.call(["/home/pi/ServoBlaster/servod"])

    def gripperMove(self, servo, angle):
        """
        Move Specified Gripper to Specified Angle (-180, 180).

        Parameters
        ----------
        servo : int
            Something
        angle : int
            Something

        """
        servo = str(servo)
        if angle == "off":
            subprocess.call("echo " + servo +
                            "=0 > /dev/servoblaster", shell=True)
        else:
            angle = float(angle)
            angle = math.atan2(math.sin(angle * 3.14 / 180),
                               math.cos(angle * 3.14 / 180)) * 180 / 3.14
            angle += 180
            # Puts in PWM range of 50-255 (55 - 250 to avoid servo extremes)
            toGripper = 55 + angle / 360 * 195
            toGripper = str(int(toGripper))
            subprocess.call("echo " + servo + "=" + toGripper +
                            " > /dev/servoblaster", shell=True)
            if servo == 0 and not angle == "off":
                time.sleep(abs(self.udPos - angle) * self.delayRatio)
                self.udPos = angle
            elif servo == 1 and not angle == "off":
                time.sleep(abs(self.rotPos - angle) * self.delayRatio)
                self.rotPos = angle
            elif servo == 2 and not angle == "off":
                time.sleep(abs(self.yPos - angle) * self.delayRatio)
                self.yPos = angle
            elif servo == 3 and not angle == "off":
                time.sleep(abs(self.cPos - angle) * self.delayRatio)
                self.cPos = angle

    def gripperInitial(self):
        """ Move gripper to a given intial condition then turn them off. """
        self.gripperMove(self.udG, -115)
        time.sleep(1)
        self.gripperMove(self.rotG, 165)
        time.sleep(1)
        self.gripperMove(self.yG, 0)
        time.sleep(1)
        self.gripperMove(self.cG, 0)
        time.sleep(1)

        self.gripperMove(self.udG, "off")
        self.gripperMove(self.rotG, "off")
        self.gripperMove(self.yG, "off")
        self.gripperMove(self.cG, "off")

    def manualInput(self):
        """ Allows the user to choose a servo and move it manually. """
        print("Servo numbers: 0 = Up and Down, 1 = Rotate Wrist, 2 = Yaw " +
              "Around the Robot, 3 = Gripper Open and Close")
        try:
            servo = int(raw_input("Enter servo number: "))
        except ValueError:
            print("That's not a number!")
            return

        if not 0 <= servo <= 3:
            print("Servo number must be in the range [0,3]!")
            return

        try:
            position = float(raw_input("Enter servo angle: "))
        except ValueError:
            print("That's not a number!")
            return

        if -180 >= position >= 180:
            print("The servo position must be in a range [-180,180] degrees!")
            return

        if servo == 0:
            print("Moving Up Down Servo!")
            # Delay should be 0.12sec per 60 deg
            self.gripperMove(servo, position)
            self.udPos = position
        elif servo == 1:
            print("Moving Wrist Rotation Servo!")
            # Delay should be 0.12sec per 60 deg
            self.gripperMove(servo, position)
            self.rotPos = position
        elif servo == 2:
            print("Moving Central Yaw Servo!")
            # Delay should be 0.12sec per 60 deg
            self.gripperMove(servo, position)
            self.yPos = position
        elif servo == 3:
            print("Moving Claw Servo!")
            # Delay should be 0.12sec per 60 deg
            self.gripperMove(servo, position)
            self.cPos = position

    def allOff(self):
        """ Turns off all the servos. """
        self.gripperMove(self.udG, "off")
        self.gripperMove(self.rotG, "off")
        self.gripperMove(self.yG, "off")
        self.gripperMove(self.cG, "off")
