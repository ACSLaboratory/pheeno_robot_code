#!/usr/bin/python

import socket
import sys
import os.path
import fcntl
import struct
import serial
import threading
import time
from Gripper import Gripper

# Move Specified Gripper to Specified Angle (-180, 180)
myGripper = Gripper()

# Make sure the script is run as sudo
if not os.geteuid() == 0:
    sys.exit("\nOnly root can run this script\n")

udG = 0
rotG = 1
yG = 2
cG = 3

# Set up the USB connection to the Arduino. This will error
# if a USB cord is not connected.
usbOptions = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
usbOptions = filter(os.path.exists, usbOptions)
assert(len(usbOptions) == 1)
usbPort = usbOptions[0]

mes = "Q"
num = 0


# Cute trick to get the local IP address from wlan0 (wireless)
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

# Recieve the data being transmitted.


def recvall(sock, length):
    data = ''
    while len(data) < length:
        more = sock.recv(length - len(data))
        if not more:
            raise EOFError('socket closed %d bytes into a %d-byte message'
                           % (len(data), length))
        data += more
    return data

# Set up and listen for commands.
# def myTCPServer(endSymbol):
#     HOST = get_ip_address('wlan0')
#     PORT = 1160
#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     s.bind((HOST, PORT))
#     s.listen(5)
#     print "Waiting for connection. Listening to port ", str(PORT)
#     client, addr = s.accept()
#     print "Accepted connection from: ", addr
#     while True:
#         client, addr = s.accept()
#         print "Accepted connection from: ", addr
#         message = recvall(client, 6)
#         client.close()
#         if message[0] == 'S':
#             client.close()
#             print "Communication Closed!"
#             break
#         print "Message Recieved: " + message #Debug print
#         # Get's the commands from the message to see if they are for the
#         # gripper.
#         parseMessage(message)
#         usb.flush()
#         usb.write(message) #Write the message recieved to the Arduino.
#         usb.flush()

# Set up and listen for commands.


def myTCPServer(endSymbol):
    HOST = get_ip_address('wlan0')
    PORT = 1160
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)
    print("Waiting for connection. Listening to port " + str(PORT))
    client, addr = s.accept()
    print("Accepted connection from: " + addr)
    while True:
        client, addr = s.accept()
        print("Accepted connection from: " + addr)
        message = []
        while ':' not in message:
            message.append(client.recv(1))
        message = ''.join(message[:])
        client.close()
        if message[0] == 'S':
            client.close()
            print("Communication Closed!")
            break
        print("Message Recieved: " + message)  # Debug print.
        # Get's the commands from the message to see if they are for the
        # gripper.
        parseMessage(message)
        usb.flush()
        usb.write(message)  # Write the message recieved to the Arduino.
        usb.flush()


# Move Specified Gripper to Specified Angle (-180,180)
def gripperMove(servo, angle):
    """
    Function containing a threading wrapper for gripperMove module of the
    Gripper class.

    Parameters
    ----------
    servo : int
        Servo on the gripper needing to be moved.
    angle : float
        Desired angle for the servo to move.

    """
    thread = threading.Thread(
        target=myGripper.gripperMove, args=(servo, angle))
    thread.start()


def parseMessage(x):
    global mes
    global num

    mes = x[0]
    num = int(x[2:-1])


def gripperClose():
    """ Close the mandibles. """
    gripperMove(3, -180)


def gripperOpen():
    """ Open the mandibles. """
    gripperMove(3, 0)


def gripperAction(strInput, numInput):
    if strInput == "A":
        if numInput == 000:
            gripperClose()
        if numInput == 001:
            gripperOpen()
    elif strInput == "H":
        gripperMove(udG, numInput)
    elif strInput == "T":
        gripperMove(yG, numInput)
    elif strInput == "Y":
        gripperMove(rotG, numInput)

# Open the communication thread
print("Communication Thread Started.")
communication = threading.Thread(target=myTCPServer, args=(":"))
communication.start()

# Main loop.
while True:
    try:
        usb = serial.Serial(usbPort, 9600, timeout=10)
        usb.open()
        # Put the gripper in the initial configuration defined in Gripper
        # class.
        myGripper.gripperInitial()
        time.sleep(2)  # give the serial time to get opened
        print("USB Connection Initialized.")
    except serial.SerialException:
        myGripper.allOff()  # Turns off all the gripper servos.
        os.system("sudo killall servod")  # Kills the servod program.
        temp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        temp.connect((get_ip_address('wlan0'), 1160))
        temp.sendall("S:")
        temp.close()
        sys.exit("USB connection not found (Is the USB cord plugged in?)")

    while True:
        try:
            gripperAction(mes, num)
            print("Command: " + mes + " Number: " + str(num))
            time.sleep(0.1)  # Thread can execute too fast for communication?
        except KeyboardInterrupt:
            myGripper.allOff()  # Turns off all the gripper servos.
            os.system("sudo killall servod")  # Kills the servod program.
            temp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            temp.connect((get_ip_address('wlan0'), 1160))
            temp.sendall("S:")
            temp.close()
            sys.exit("Successfully Exited Main Loop")

    # End script if inner while loop ends.
    break
