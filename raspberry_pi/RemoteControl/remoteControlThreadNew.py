# Import System Libraries
import socket
import sys
import os.path
import fcntl
import struct
import serial
import threading
import time
from Gripper import Gripper


def get_ip_address(ifname):
    """ Get local IP address from wlan0. """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(), 0x8915, struct.pack('256s', ifname[:15]))[20:24])


def recieve_all(sock, length):
    """ Recieve the data being transmitted. (Formerlly recvall) """
    data = ''
    while len(data) < length:
        more = sock.recv(length - len(data))
        if not more:
            raise EOFError('Socket closed %d bytes into a %d-byte message'
                           % (len(data), length))
        data += more
    return data


def parse_message(x):
    global mes
    global num
    mes = x[0]
    num = int(x[2:-1])


class TCPServerMotor(threading.Thread):
    """
    A class thread for incoming data from the base station.

    Parameters
    ----------
    usb_port : str
        Specified usb port that the Pheeno's Raspberry Pi will access the
        Pheeno's Arduino.
    end_delimiter : str (Default: ':')
        Desired delimiter to parse incoming Pheeno movement data.
    port : int (Default: 1160)
        Specific port for the tcp server to connect to Pheeno.

    Attributes
    ----------
    HOST : str
        IP address for the TCP server.
    PORT : str
        Port for the TCP server.
    end_delimiter : str
        Delimiter used to determine separate commands.
    usb_port_serial : str
        The usb port that the Pheeno's Raspberry Pi will access the
        Pheeno's Arduino.
    is_activated : bool
        The thread control statement. When false, the thread will end.

    Methods
    -------
    parse_message(message)
        Parses the incoming message from the central computer.

    """
    def __init__(self, usb_port, end_delimiter=":", port=1160):
        threading.Thread.__init__(self)

        self.HOST = get_ip_address('wlan0')
        self.PORT = port
        self.end_delimiter = end_delimiter
        self.usb_port_serial = usb_port
        self.is_activated = True

    def run(self):
        """ Thread RUN call. """
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))
        s.listen(5)

        print("Waiting for connection. Listening to port " + str(self.PORT))
        client, addr = s.accept()
        print("Accepted connection from: " + addr[0])

        # Initialize USB Serial Socket
        usb_serial = serial.Serial(self.usb_port_serial, 9600, timeout=10)
        usb_serial.open()
        time.sleep(2)
        print("USB Connection Initialized.")

        while self.is_activated:
            try:
                client, addr = s.accept()
                print("Accepted connection from: " + addr[0])
                message = []
                while self.end_delimiter not in message:
                    message.append(client.recv(1))
                message = ''.join(message[:])
                client.close()
                if message[0] == 'S':
                    client.close()
                    print("Communication Closed!")
                    break
                print("Message Received: " + message)
                parse_message(message)
                usb_serial.flush()
                usb_serial.write(message)
                usb_serial.flush()
                time.sleep(0.1)

            except KeyboardInterrupt:
                # Stop the loop.
                self.is_activated = False
                time.sleep(0.1)

        print("Connection Thread Ended.")
        return

    @staticmethod
    def parse_message(message):
        """ Separates message into specific values. """
        mess = message[0]
        number = int(message[2:-1])
        print("Command: " + mess + " Number: " + str(number))

if __name__ == '__main__':

    mes = "Q"
    num = 0

    # Makes sure the script is run as sudo.
    if not os.geteuid() == 0:
        sys.exit("\nMust be root to run this script!\n")

    # Set up the USB Connection to the Arduino
    usb_options = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    usb_options = filter(os.path.exists, usb_options)
    assert(len(usb_options) == 1)  # Exist if no usb connection.
    usb_port = usb_options[0]

    # Open the communication thread.
    print("Communication Thread Started.")
    communication_motor = tcp_server_motor(usb_port)
    communication_motor.start()

    # Set up and start gripper thread. /--- EXPERIMENTAL ---/
    print("Gripper Thread Started.")
    pheeno_gripper = Gripper()
    communication_gripper = threading.Thread(target=myGripper.gripperMove,
                                             args=(servo, angle))
    communication_gripper.start()
