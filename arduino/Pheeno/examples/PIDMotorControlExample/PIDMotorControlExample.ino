/* This script gives an introduction to using encoder feedback on Pheeno's motors.
In this code we will make Pheeno drive straight at a given velocity.
A premade PID controller has been made to control the individual motor speeds
and can be accessed using the Pheeno.PIDMotorControl() function. The PID feedback
relies solely on the encoders so wheel slip can cause to robot to diverge 
from a straight line path! The gains of the PID can be adjusted in the Pheeno.cpp file.
*/

#include "Pheeno.h" // If you want to use Pheeno's premade routines import this!

Pheeno myRobot = Pheeno(0); // The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD)

float desVel = 10; //The linear velocity we want the robot to go in cm/s

void setup(){
  myRobot.SETUP(); //This must be included in every script! It sets up all the pins on the robot!
  desVel = desVel * 2/myRobot.wheelDiameter;//Converts the linear velocity to rotational velocity for the controller.
}

void loop() {
  myRobot.PIDMotorControl(desVel,desVel);
}
