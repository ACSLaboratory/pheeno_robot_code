# System Imports
import time

# Import Pheeno Class
import Pheeno

if __name__ == "__main__":
    # Instantiate the Pheeno and Setup.
    robot = Pheeno.Pheeno('192.168.119.31', 1160)
    print('Setup')
    robot.setup()

    # Test forward movement.
    time.sleep(3)
    print('Move Forward.')
    robot.move_forward(200)
    time.sleep(2)
    print('Stop')
    robot.stop()
    time.sleep(2)

    # Test left movement.
    print('Move Backwards')
    robot.move_left(200)
    time.sleep(2)
    print('Stop')
    robot.stop()
    time.sleep(2)

    # Test right movement.
    print('Move Right')
    robot.move_right(200)
    time.sleep(2)
    print('Stop')
    robot.stop()
    time.sleep(2)

    # Test reverse movement
    print('Move Back')
    robot.move_reverse(200)
    time.sleep(2)
    print('Stop')
    robot.stop()
    time.sleep(2)

    # Test gripper altitude
    print("Servo Altitude")
    robot.g_servo_altitude(-100)
    time.sleep(1)
    print("Servo Yaw")
    robot.g_servo_yaw(-100)
    time.sleep(1)
    print("Servo Roll")
    robot.g_servo_roll(-100)
    time.sleep(1)

    # Stop Test
    print('Stop Robot')
    robot.stop()
