# Import Tkinter Libraries
# from PIL import Image, ImageTk
from Tkinter import *
from tkMessageBox import *

# Import System Libraries
import time
import socket
import sys


def left(event):
    """
    Event socket to move Pheeno left.

    When the GUI button to move the Pheeno left is pressed, the following
    'left' function calls the sendCommand function, with the proper arguments,
    to communicate with Pheeno. The Pheeno will then move left.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    # Generate Command to Send
    to_send = "L;" + str(speed.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if speed.get() < 100 and speed.get() >= 10:
    # 	toSend = "L;0" + str(speed.get())
    # elif speed.get() < 10:
    # 	toSend = "L;00" + str(speed.get())
    # else:
    # 	toSend = "L;" + str(speed.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def right(event):
    """
    Event socket to move Pheeno right.

    When the GUI button to move the Pheeno right is pressed, the following
    'right' function calls the sendCommand function, with the proper arguments,
    to communicate with Pheeno. The Pheeno will then move right.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    # Generate Command to Send
    to_send = "R;" + str(speed.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if speed.get() < 100 and speed.get() >= 10:
    # 	toSend = "R;0" + str(speed.get())
    # elif speed.get() < 10:
    # 	toSend = "R;00" + str(speed.get())
    # else:
    # 	toSend = "R;" + str(speed.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def foward(event):
    """
    Event socket to move Pheeno forward.

    When the GUI button to move the Pheeno forward is pressed, the following
    'forward' function calls the sendCommand function, with the proper
    arguments, to communicate with Pheeno. The Pheeno will then move forward.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    # Generate Command to Send
    to_send = "F;" + str(speed.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if speed.get() < 100 and speed.get() >= 10:
    # 	toSend = "F;0" + str(speed.get())
    # elif speed.get() < 10:
    # 	toSend = "F;00" + str(speed.get())
    # else:
    # 	toSend = "F;" + str(speed.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def reverse(event):
    """
    Event socket to move Pheeno backwards.

    When the GUI button to move the Pheeno backwards is pressed, the following
    'reverse' function calls the sendCommand function, with the proper
    arguments, to communicate with Pheeno. The Pheeno will then move in
    reverse.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    to_send = "B;" + str(speed.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if speed.get() < 100 and speed.get() >= 10:
    # 	toSend = "B;0" + str(speed.get())
    # elif speed.get() < 10:
    # 	toSend = "B;00" + str(speed.get())
    # else:
    # 	toSend = "B;" + str(speed.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def servoUp(event):
    """
    Event socket to move gripper arm up the prismatic belt.

    Actuation of the gripper module. Sends the proper ASCII characters to move
    the gripper arm up the prismatic belt by using the sendCommand function.
    The servo has a limit of -180, any actions greater than this will be
    artificially pushed down to -182.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if altServo.get() <= -180:
        altServo.set(-180)

    else:
        altServo.set(altServo.get() - 2)

    # Generate Command to Send
    to_send = "H;" + str(altServo.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if altServo.get() < 100 and altServo.get() >= 10:
    # 	toSend = "H;0" + str(altServo.get())
    # elif altServo.get() < 10:
    # 	toSend = "H;00" + str(altServo.get())
    # else:
    # 	toSend = "H;" + str(altServo.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def servoDown(event):
    """
    Event socket to move gripper arm down the prismatic joint.

    Actuation of the gripper module. Sends the proper ASCII characters to move
    the gripper arm down the prismatic belt by using the sendCommand function.
    The servo has a limit of -50, any actions less than this will be
    artificially pushed up to -48.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if altServo.get() >= -50:
        altServo.set(-50)

    else:
        altServo.set(altServo.get() + 2)

    # Generate Command to Send
    to_send = "H;" + str(altServo.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if altServo.get() < 100 and altServo.get() >= 10:
    # 	toSend = "H;0" + str(altServo.get())
    # elif altServo.get() < 10:
    # 	toSend = "H;010"
    # else:
    # 	toSend = "H;" + str(altServo.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def servoYR(event):
    """
    Event socket to yaw the gripper arm right.

    Actuation of the gripper module. Sends the proper ASCII characters to yaw
    the gripper arm right by using the sendCommand function. The
    servo has a limit of 180, any actions less than this will be artificially
    pushed up to 185.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if yawServo.get() >= 180:
        yawServo.set(180)

    else:
        yawServo.set(yawServo.get() + 5)

    # Generate Command to Send
    to_send = "T;" + str(yawServo.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if yawServo.get() >= 10 and yawServo.get() < 100:
    # 	toSend = "T;0" + str(yawServo.get())
    # elif yawServo.get() > 0 and yawServo.get() < 10:
    # 	toSend = "T;010"
    # else:
    # 	toSend = "T;" + str(yawServo.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def servoYL(event):
    """
    Event socket to yaw the gripper arm left.

    Actuation of the gripper module. Sends the proper ASCII characters to yaw
    the gripper arm left by using the sendCommand function. The servo has a
    limit of -180, any actions greater than this will be artificially
    pushed down to -185.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if yawServo.get() <= -180:
        yawServo.set(-180)
    else:
        yawServo.set(yawServo.get() - 5)

    # Generate Command to Send
    to_send = "T;" + str(yawServo.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if yawServo.get() >= 10 and yawServo.get() < 100:
    # 	toSend = "T;0" + str(yawServo.get())
    # elif yawServo.get() > 0 and yawServo.get() < 10:
    # 	toSend = "T;010"
    # else:
    # 	toSend = "T;" + str(yawServo.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def servoRR(event):
    """
    Event socket to roll the gripper arm right.

    Actuation of the gripper module. Sends the proper ASCII characters to roll
    the gripper arm right by using the sendCommand function. The servo has a
    limit of 180, any actions less than this will be artificially pushed up
    to 185.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if rollServo.get() >= 180:
        rollServo.set(180)

    else:
        rollServo.set(rollServo.get() + 5)

    # Generate Command to Send
    to_send = "Y;" + str(rollServo.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if rollServo.get() >= 10 and rollServo.get() < 100:
    # 	toSend = "Y;0" + str(rollServo.get())
    # elif rollServo.get() > 0 and rollServo.get() < 10:
    # 	toSend = "Y;010"
    # else:
    # 	toSend = "Y;" + str(rollServo.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def servoRL(event):
    """
    Event socket to roll the gripper arm left.

    Actuation of the gripper module. Sends the proper ASCII characters to roll
    the gripper arm left by using the sendCommand function. The servo has a
    limit of -180, any actions greater than this will be artificially
    pushed down to -185.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if rollServo.get() <= -180:
        rollServo.set(-180)

    else:
        rollServo.set(rollServo.get() - 5)

    # Generate Command to Send
    to_send = "Y;" + str(rollServo.get())
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")
    # if rollServo.get() >= 10 and rollServo.get() < 100:
    # 	toSend = "Y;0" + str(rollServo.get())
    # elif rollServo.get() > 0 and rollServo.get() < 10:
    # 	toSend = "Y;010"
    # else:
    # 	toSend = "Y;" + str(rollServo.get())
    # sendCommand(toSend + ":")
    # print("Command is " + toSend + ":")


def grab(event):
    """
    Event socket for gripper to grab an object.

    This operation is boolean the gripper will either be in a state of open or
    closed. When called by the GUI button, the grab function will check the
    state of the gripper and then give the opposite. The sendCommand function
    will then relay the state change over the socket connection.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    if gbool.get() == 1:
        gbool.set(0)
        to_send = "A;" + str('{:03d}'.format(0))
    else:
        gbool.set(1)
        to_send = "A;001"
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")


def stop(event):
    """
    Event socket to stop commands to the Pheeno.

    Parameters
    ----------
    event : str
        A Tkinter signal string containing information from the widget that
        calls this function. It is not used in this function.

    """
    to_send = "Z;" + str('{:03d}'.format(0))
    sendCommand(to_send + ":")
    print("Command is " + to_send + ":")


def setup():
    """ Default values for the Pheeno. """
    speed.set(150)
    altServo.set(-115)
    yawServo.set(0)
    rollServo.set(-165)
    gbool.set(1)


def help():
    """ Display's help info for Tkinter menu. """
    showinfo('Help Menu', 'Use the arrow keys to move pheeno. "W" raises the'
             ' gripper, "S" lowers it. "A" will cause the gripper to rotate'
             ' to the right. "D" will cause it to rotate left. "Q" and "E"'
             ' will roll the gripper.')


# Server/Client Stuff
HOST = sys.argv.pop() if len(sys.argv) == 2 else '127.0.0.1'
PORT = 1160


def sendCommand(command):
    """
    Send Pheeno commands through a socket connection.

    Opens a socket and sends commands (in ASCII format) given by the user. The
    socket will then safely close.

    Parameters
    ----------
    command : str
        An ASCII string that commands the Pheeno to do a specific task.

    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.sendall(command)
    s.close()


""" Master GUI Window """
master = Tk()
master.geometry("480x720+10+0")  # width x hight + x_offset + y_offset
Label(master, text='Pheeno Controller!', font=("Helvetica", 36),
      fg="red").place(x=10, y=10)
gbool = BooleanVar(master)

# logo = PhotoImage(file="C:/Users/ACSlab/Desktop/test.gif")
# Label(master, image=logo).place(x=30,y=450)

""" Speed Widget """
speed = Scale(master, from_=0, to=255, length=200, tickinterval=50,
              orient=HORIZONTAL)
speed.set(150)
speed.place(x=180, y=80)
Label(master, text='Robot Speed', font=("Helvetica", 16)).place(x=30, y=95)

""" Gripper Altitude Widget """
altServo = Scale(master, from_=-50, to=-180, length=200, tickinterval=30,
                 orient=HORIZONTAL)
altServo.set(-115)
altServo.place(x=180, y=160)
Label(master, text='Gripper Altitude',
      font=("Helvetica", 16)).place(x=30, y=175)

""" Yaw Servo Widget """
yawServo = Scale(master, from_=-180, to=180, length=200, tickinterval=60,
                 orient=HORIZONTAL)
yawServo.set(0)
yawServo.place(x=180, y=240)
Label(master, text='Gripper Yaw', font=("Helvetica", 16)).place(x=30, y=255)

""" Roll Servo Widget """
rollServo = Scale(master, from_=-180, to=180, length=200, tickinterval=60,
                  orient=HORIZONTAL)
rollServo.set(-165)
rollServo.place(x=180, y=320)
Label(master, text='Gripper Roll', font=("Helvetica", 16)).place(x=30, y=335)

""" Quit Button Widget """
quitButton = Button(master, text='Quit', bg="red", font=("Helvetica", 16),
                    command=master.quit)
quitButton.place(x=10, y=670)

""" Setup Button Widget """
setupButton = Button(master, text='Setup', bg="blue", font=("Helvetica", 16),
                     command=setup)
setupButton.place(x=80, y=670)

""" Help Button Widget """
helpButton = Button(master, text='HELP!', bg="orange", font=("Helvetica", 26),
                    command=help)
helpButton.place(x=350, y=450)

""" Move Forward Button Widget """
moveFButton = Button(None, text='^', bg="black", fg="white",
                     font=("Helvetica", 16), height=1, width=2)
moveFButton.place(x=380, y=620)

""" Move Back Button Widget """
moveBButton = Button(None, text='v', bg="black", fg="white",
                     font=("Helvetica", 16), height=1, width=2)
moveBButton.place(x=380, y=670)

""" Move Right Button Widget """
moveRButton = Button(None, text='>', bg="black", fg="white",
                     font=("Helvetica", 16), height=1, width=2)
moveRButton.place(x=420, y=670)

""" Move Left Button Widget """
moveLButton = Button(None, text='<', bg="black", fg="white",
                     font=("Helvetica", 16), height=1, width=2)
moveLButton.place(x=340, y=670)

""" Gripper Up Button Widget """
gripperUButton = Button(None, text='W', bg="black", fg="white",
                        font=("Helvetica", 16), height=1, width=2)
gripperUButton.place(x=250, y=620)

""" Gripper Down Button Widget """
gripperDButton = Button(None, text='S', bg="black", fg="white",
                        font=("Helvetica", 16), height=1, width=2)
gripperDButton.place(x=250, y=670)

""" Gripper Yaw Left Button Widget """
gripperLYButton = Button(None, text='A', bg="black", fg="white",
                         font=("Helvetica", 16), height=1, width=2)
gripperLYButton.place(x=210, y=670)

""" Gripper Yaw Right Button Widget """
gripperRYButton = Button(None, text='D', bg="black", fg="white",
                         font=("Helvetica", 16), height=1, width=2)
gripperRYButton.place(x=290, y=670)

""" Gripper Roll Right Button Widget """
gripperRRButton = Button(None, text='E', bg="black", fg="white",
                         font=("Helvetica", 16), height=1, width=2)
gripperRRButton.place(x=290, y=620)

""" Gripper Roll Left Button Widget """
gripperLRButton = Button(None, text='Q', bg="black", fg="white",
                         font=("Helvetica", 16), height=1, width=2)
gripperLRButton.place(x=210, y=620)

""" Gripper Grab Button Widget """
grabButton = Button(master, text='GRAB!', bg="purple", fg="orange",
                    font=("Helvetica", 26))
grabButton.place(x=200, y=450)

moveFButton.bind('<Button-1>', foward)
moveBButton.bind('<Button-1>', reverse)
moveRButton.bind('<Button-1>', right)
moveLButton.bind('<Button-1>', left)

gripperUButton.bind('<Button-1>', servoUp)
gripperDButton.bind('<Button-1>', servoDown)
gripperLYButton.bind('<Button-1>', servoYL)
gripperRYButton.bind('<Button-1>', servoYR)
gripperLRButton.bind('<Button-1>', servoRL)
gripperRRButton.bind('<Button-1>', servoRR)

grabButton.bind('<Button-1>', grab)


master.bind('<Left>', left)
master.bind('<Right>', right)
master.bind('<Up>', foward)
master.bind('<Down>', reverse)

master.bind('<w>', servoUp)
master.bind('<s>', servoDown)
master.bind('<a>', servoYL)
master.bind('<d>', servoYR)
master.bind('<q>', servoRL)
master.bind('<e>', servoRR)

master.bind('<KeyRelease>', stop)

master.bind('<g>', grab)


mainloop()
