# System Imports
import socket


class Pheeno(object):
    """
    Pheeno Robot Class

    An instance of the Pheeno Robot class that allows for direct commanding or
    application of swarm control schemes in other python scripts.

    Parameters
    ----------
    host : str
        The host IP connection value.
    port : int
        The port value.
    robot_type : int (Default: 0)
        The Pheeno can be of three different types, which can be determined
        by the physical Pheeno.

    Attributes
    ----------
    speed : int
        The current speed of the Pheeno robot instance.
    servo_altitude : int
        The current servo altitude of the Pheeno's Gripper.
    servo_yaw : int
        The current servo yaw of the Pheeno's Gripper.
    servo_roll : int
        The current servo roll of the Pheeno's Gripper.

    Methods
    -------
    move_left(desired_speed)
        Moves the Pheeno left at the desired speed.
    move_right(desired_speed)
        Moves the Pheeno right at the desired speed.
    move_forward(desired_speed)
        Moves the Pheeno forward at the desired speed.
    move_reverse(desired_speed)
        Moves the Pheeno backwards at the desired speed.
    g_servo_altitude(desired_altitude)
        Moves the gripper servo at the specified altitude.
    g_servo_altitude_up() && g_servo_altitude_down()
        Incremental motion of the altitude servo on the Gripper.
    g_servo_yaw(desired_yaw)
        Yaws the gripper servo at the specified yaw.
    g_servo_yaw_right() && g_servo_yaw_left()
        Incremental motion of the yaw servo on the Gripper.
    g_servo_roll(desired_roll)
        Rolls the gripper servo at the specified roll.
    g_servo_roll_right() && g_servo_roll_left()
        Incremental motion of the roll servo on the Gripper.
    stop()
        Stops the Pheeno.
    send_command(command)
        Opens a socket connection and creates
    setup()
        Reverts servo values back to their defaults and actuates those new
        values. Should be run at the beginning of every Pheeno use case.

    """

    def __init__(self, host, port, robot_type=0):
        # User controlled values
        self.speed = 150
        self.servo_altitude = -115
        self.servo_yaw = 0
        self.servo_roll = -165

        # Booleans and Attributes
        self.is_socket_open = False
        self.is_gripper_open = True

        # Network Attributes
        self.host = host
        self.port = port

        # Private MIN and MAX servo values.
        self.__speed_values = [0, 255]
        self.__alt_servo_values = [-180, -50]
        self.__yaw_servo_values = [-180, 180]
        self.__roll_servo_values = [-180, 180]

        # Robot Type
        self.__robot_type = robot_type

    def move_left(self, desired_speed):
        """
        Move the Pheeno Left.

        Parameters
        ----------
        desired_speed : int
            The desired speed of the Pheeno, this can only be from 0 to 255.

        """
        if self.__speed_values[0] > desired_speed > self.__speed_values[1]:
            print("ERROR")

        else:
            self.speed = desired_speed
            command = "L;" + str(int(desired_speed)) + ":"
            self.send_command(command)

    def move_right(self, desired_speed):
        """
        Move the Pheeno Right.

        Parameters
        ----------
        desired_speed : int
            The desired speed of the Pheeno, this can only be from 0 to 255.

        """
        if self.__speed_values[0] > desired_speed > self.__speed_values[1]:
            print("ERROR")

        else:
            self.speed = desired_speed
            command = "R;" + str(int(desired_speed)) + ":"
            self.send_command(command)

    def move_forward(self, desired_speed):
        """
        Move the Pheeno Forward.

        Parameters
        ----------
        desired_speed : int
            The desired speed of the Pheeno, this can only be from 0 to 255.

        """
        if self.__speed_values[0] > desired_speed > self.__speed_values[1]:
            print("ERROR")

        else:
            self.speed = desired_speed
            command = "F;" + str(int(desired_speed)) + ":"
            self.send_command(command)

    def move_reverse(self, desired_speed):
        """
        Move the Pheeno in Reverse.

        Parameters
        ----------
        desired_speed : int
            The desired speed of the Pheeno, this can only be from 0 to 255.

        """
        if self.__speed_values[0] > desired_speed > self.__speed_values[1]:
            print("ERROR")

        else:
            self.speed = desired_speed
            command = "B;" + str(int(desired_speed)) + ":"
            self.send_command(command)

    def g_servo_altitude(self, desired_altitude):
        """
        Move the Pheeno's Gripper up and down by a desired altitude value.

        Parameters
        ----------
        desired_altitude : int
            The desired altitude for the gripper to move. The operating range
            can only be between -180 to -50.

        """
        if (self.__alt_servo_values[0] <= desired_altitude <=
                self.__alt_servo_values[1]):
            self.servo_altitude = desired_altitude
            command = "H;" + str(int(desired_altitude)) + ":"
            self.send_command(command)

        else:
            print("ERROR")

    def g_servo_altitude_up(self):
        """ Move the Pheeno's Gripper altitude up by 2. """
        if self.servo_altitude <= self.__alt_servo_values[0]:
            self.servo_altitude = self.__alt_servo_values[0]

        else:
            self.servo_altitude -= 2

        # Generate Command to Send
        command = "H;" + str(self.servo_altitude) + ":"
        self.send_command(command)

    def g_servo_altitude_down(self):
        """ Move the Pheeno's Gripper altitude down by 2. """
        if self.servo_altitude >= self.__alt_servo_values[1]:
            self.servo_altitude = self.__alt_servo_values[1]

        else:
            self.servo_altitude += 2

        # Generate Command to Send
        command = "H;" + str(self.servo_altitude) + ":"
        self.send_command(command)

    def g_servo_yaw(self, desired_yaw):
        """
        Yaw the Pheeno's Gripper by a desired yaw value.

        Parameters
        ----------
        desired_yaw : int
            The desired yaw for the gripper to move. The operating range of
            this is -180 to 180.

        """
        if (self.__yaw_servo_values[0] <= desired_yaw <=
                self.__yaw_servo_values[1]):
            self.servo_yaw = desired_yaw
            command = "T;" + str(int(desired_yaw)) + ":"
            self.send_command(command)

        else:
            print("ERROR")

    def g_servo_yaw_right(self):
        """ Yaw the Pheeno's Gripper right by a 5 degree increment. """
        if self.servo_yaw >= self.__yaw_servo_values[1]:
            self.servo_yaw = self.__yaw_servo_values[1]

        else:
            self.servo_yaw += 5

        # Generate Command to Send
        command = "T;" + str(self.servo_yaw) + ":"
        self.send_command(command)

    def g_servo_yaw_left(self):
        """ Yaw the Pheeno's Gripper left by a 5 degree increment. """
        if self.servo_yaw <= self.__yaw_servo_values[0]:
            self.servo_yaw = self.__yaw_servo_values[0]

        else:
            self.servo_yaw -= 5

        # Generate Command to Send
        command = "T;" + str(self.servo_yaw) + ":"
        self.send_command(command)

    def g_servo_roll(self, desired_roll):
        """
        Roll the Pheeno's Grippe by a desired roll value.

        Parameters
        ----------
        desired_roll : int
            The desired roll for the gripper to move. The operating range of
            this is -180 to 180.

        """
        if (self.__roll_servo_values[0] <= desired_roll <=
                self.__roll_servo_values[1]):
            self.servo_roll = desired_roll
            command = "Y;" + str(int(desired_roll)) + ":"
            self.send_command(command)

        else:
            print("ERROR")

    def g_servo_roll_right(self):
        """ Roll the Pheeno's Gripper right by a 5 degree increment. """
        if self.servo_roll >= self.__roll_servo_values[1]:
            self.servo_roll = self.__roll_servo_values[1]

        else:
            self.servo_roll += 5

        # Generate Command to Send
        command = "Y;" + str(self.servo_roll) + ":"
        self.send_command(command)

    def g_servo_roll_left(self):
        """ Roll the Pheeno's Gripper left by a 5 degree increment. """
        if self.servo_roll <= self.__roll_servo_values[0]:
            self.servo_roll = self.__roll_servo_values[0]

        else:
            self.servo_roll -= 5

        # Generate Command to Send
        command = "Y;" + str(self.servo_roll) + ":"
        self.send_command(command)

    def grab(self):
        """
        Use the Pheeno's claw to grab an object.
        """
        if self.is_gripper_open:
            self.is_gripper_open = False
            command = "A;" + str('{:03d}'.format(0)) + ":"
        else:
            self.is_gripper_open = True
            command = "A;001:"
        self.send_command(command)

    def stop(self):
        """ Stop commands to the Pheeno. """

        command = "Z;" + str('{:03d}'.format(0)) + ":"
        self.send_command(command)

    def send_command(self, command):
        """
        Send Pheeno commands through a socket connection.

        Opens a socket and sends commands (in ASCII format) given by the
        user. The socket will then safely close.

        Parameters
        ----------
        command : str
            An ASCII string that commands the Pheeno to do a specific task.

        """
        # HOST = sys.argv.pop() if len(sys.argv) == 2 else '127.0.0.1'
        # self.host = '192.168.119.31'
        # self.port = 1160
        # PORT = 12345
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.host, self.port))
        s.sendall(command)
        s.close()

    def setup(self):
        """ Default attribute values. """
        self.speed = 150
        self.servo_altitude = -115
        self.servo_yaw = 0
        self.servo_roll = -165
        self.is_gripper_open = True

        self.g_servo_altitude(self.servo_altitude)
        self.g_servo_yaw(self.servo_yaw)
        self.g_servo_roll(self.servo_roll)

    def __str__(self):
        """ Display the state of the Pheeno Robot instance. """
        print('Pheeno State')
        print('------------------------')
        print('Pheeno Speed: %d' % self.speed)
        print('Pheeno Gripper Altitude: %d' % self.servo_altitude)
        print('Pheeno Gripper Yaw: %d' % self.servo_yaw)
        print('Pheeno Gripper Roll: %d' % self.servo_roll)
        print('Pheeno Gripper State: %s' %
              'Open' if self.is_gripper_open else 'Closed')
