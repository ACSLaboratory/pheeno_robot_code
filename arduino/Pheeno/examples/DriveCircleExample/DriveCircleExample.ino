/* This script gives an introduction to making Pheeno turn with encoder
  feedback. This uses the same PID controller as PIDMotorControlExample.
  In this example Pheeno will drive in a circle of radius R at angular speed
  w.
*/

#include "Pheeno.h" // If you want to use Pheeno's premade routines import this!
Pheeno myRobot = Pheeno(0);

float desW = 3.14; //The angular velocity Pheeno will traverse the circle (rad/s)! CCW direction is positive velocity!
float R = 10; //The radius of the circle Pheeno will be on (cm).

void setup(){
  myRobot.SETUP(); //This must be included in every script! It sets up all the pins on the robot!
}

void loop(){
  /* The inputs to this function are:
  (Desired Radius (cm), Desired Angular Velocity (rad/s)).*/
  myRobot.rotateAboutICC(R, desW);
}
