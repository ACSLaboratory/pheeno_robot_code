/* This script gives an introduction to making Pheeno drive open loop.
There is no feedback, motors move at the given input but can be prone
to errors in manufacturing and environment. Everything will be done with
delays, however, it should be noted you cannot use delays 
if you require sensor feedback.
*/

#include "Pheeno.h" // If you want to use Pheeno's premade routines import this!
Pheeno myRobot = Pheeno(0); // The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD)


void setup(){
  myRobot.SETUP(); //This must be included in every script! It sets up all the pins on the robot!
}

void loop(){
  myRobot.forward(150);//Pheeno drives forward.
  delay(5000);//Wait for 5 seconds
  myRobot.turnRight(70);//Pheeno turns right.
  delay(2000);//Wait for 2 seconds.
  myRobot.turnLeft(70);//Pheeno rotates left.
  delay(2000);//Wait for 2 seconds.
  myRobot.reverse(150);//Pheeno drives backwards.
  delay(5000);//Wait for 5 seconds
  myRobot.brake();//Brake the motors.
  delay(3000);//Wait for 3 seconds.
}
