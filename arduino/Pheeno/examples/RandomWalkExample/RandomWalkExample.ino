/* This script gives an introduction to making Pheeno drive a random walk. It uses delays and
arduino PWM units (int, 0-255) for speed.
*/

#include "Pheeno.h" // If you want to use Pheeno's premade routines import these libraries!!

Pheeno myRobot = Pheeno(0); // The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD)

void setup(){
  myRobot.SETUP(); //This must be included in every script! It sets up all the pins on the robot!
}

void loop(){
  float turnDelayTime = random(0,2000); //randomly generate the ms of delay for the turn.
  if (random(0,2) < 1){ /*The arduino random function is inclusive on the lower bound and exclusive
                        on the upper bound. Thus this randomly generates 0 and 1 and creates a 
                        coin flip whether to move left or right.*/
    myRobot.turnLeft(150); //Pheeno rotates left about its center at a given speed (range 0 to 255).
    delay(turnDelayTime); //Wait a random amount of time defined above.
  }
  else{
    myRobot.turnRight(150);//Pheeno rotates right about its center at a given speed (range 0 to 255).
    delay(turnDelayTime);//Wait a random amount of time defined above.
  }
  
  float runDelayTime = 1000;//ms of delay for the run (you can make this random if you desire!).
  myRobot.forward(120);//Pheeno moves forward at a given speed (range 0 to 255).
  delay(runDelayTime);//Wait a random amount of time defined above.
}
