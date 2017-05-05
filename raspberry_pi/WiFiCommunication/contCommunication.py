import time
import serial
import socket
import os.path
import sys
from Gripper import Gripper

myGripper = Gripper()

PORT = '12345'
usbOptions = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
usbOptions = filter(os.path.exists, usbOptions)
assert(len(usbOptions) == 1)
usbPort = usbOptions[0]

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('0.0.0.0', 12345))
s.listen(1)
client, _ = s.accept()

while True:
    myGripper.gripperMove(3, 100, 0.1)
    try:
        usb = serial.Serial(usbPort, 9600, timeout=10)
        usb.open()
        time.sleep(2)  # give the serial time to get opened
    except serial.SerialException:
        print("Could not connect to requested port")
        time.sleep(1)
        continue
    while True:
        message = []
        while '!' not in message:
            message.append(client.recv(1))
        message = ''.join(message[:])
        print(message)
        usb.flush()
        usb.write(message)
        usb.flush()
        message = usb.readline()
        print("Message was: " + str(message))
        if (message == ''):
            usb.flush()
            usb.write('S!')
            usb.flush()
            usb.close()
            sys.exit("Lost Connection")

        if (message[0] == "S"):
            usb.flush()
            usb.close()
            myGripper.allOff()
            sys.exit("Successfully Exited")
