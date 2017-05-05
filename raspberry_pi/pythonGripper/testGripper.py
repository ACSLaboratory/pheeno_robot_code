from Gripper import Gripper
import os

if __name__ == '__main__':
    myGripper = Gripper()

    while True:
        try:
            myGripper.manualInput()

        except KeyboardInterrupt:
            myGripper.allOff()
            os.system("sudo killall servod")
            print("Closed Fine!")
            break
