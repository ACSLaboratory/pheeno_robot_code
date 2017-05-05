/* This script gives an introduction to making Pheeno drive a random walk. 
While doing the random walk, it will avoid obstacles.
*/

#include "Pheeno.h" // If you want to use Pheeno's premade routines import these libraries!!

Pheeno myRobot = Pheeno(1); // The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD)

float rangeToAvoid = 10; //Distance in CM at which Pheeno will avoid obstacles.
float compareTime; //Compare time in ms we will base our while loops off of. Initialized in the set up!

void setup(){
  myRobot.SETUP(); //This must be included in every script! It sets up all the pins on the robot!
  compareTime = millis(); //Compare time in ms we will base our while loops off of.
}

void loop(){
  float turnDelayTime = random(0,2000); //randomly generate the ms of delay for the turn.
  if (random(0,2) < 1){ /*The arduino random function is inclusive on the lower bound and exclusive
                        on the upper bound. Thus this randomly generates 0 and 1 and creates a 
                        coin flip whether to move left or right.*/
    myRobot.turnLeft(150); //Pheeno rotates left about its center at a given speed (range 0 to 255).
    while(millis() - compareTime < turnDelayTime){ //Wait a random amount of time defined above.
      avoidObstacles();
    }
    compareTime=millis();
  }
  else{
    myRobot.turnRight(150);//Pheeno rotates right about its center at a given speed (range 0 to 255).
    while(millis() - compareTime < turnDelayTime){ //Wait a random amount of time defined above.
      avoidObstacles();
    }
    compareTime=millis();
  }
  
  float runDelayTime = 1000;//ms of delay for the run (you can make this random if you desire!).
  myRobot.forward(120);//Pheeno moves forward at a given speed (range 0 to 255).
  while(millis() - compareTime < runDelayTime){ //Wait a random amount of time defined above.
      avoidObstacles();
  }
  compareTime=millis();
}

void avoidObstacles(){
  myRobot.readIR();
  int collisionRotateSpeed = 150; //Speed at which to rotate away from collisions (In arduino PWM units (int, 0 to 255)).
  if (myRobot.CDistance < rangeToAvoid){
   if(abs((myRobot.RDistance - myRobot.LDistance) < 5 || (myRobot.RDistance > rangeToAvoid && myRobot.LDistance > rangeToAvoid)) )
    {
      if(random(0,2) < 1)
      {
         myRobot.turnLeft(collisionRotateSpeed);
      }
      else
      {
         myRobot.turnRight(collisionRotateSpeed);
      }
    } 
    if (myRobot.RDistance < myRobot.LDistance)
    {
      myRobot.turnLeft(collisionRotateSpeed);
    }
    else
    {
      myRobot.turnRight(collisionRotateSpeed);
    }
  }
  else if (myRobot.RFDistance < rangeToAvoid && myRobot.LFDistance < rangeToAvoid){
     if(random(0,2) < 1){
       myRobot.turnLeft(collisionRotateSpeed);
     }
     else{
       myRobot.turnRight(collisionRotateSpeed);
     } 
  }
  else if (myRobot.RFDistance < rangeToAvoid){
    myRobot.turnLeft(collisionRotateSpeed);
  }
  else if (myRobot.LFDistance < rangeToAvoid){
    myRobot.turnRight(collisionRotateSpeed);
  }
  else if (myRobot.LDistance < rangeToAvoid){
    myRobot.turnRight(collisionRotateSpeed);
  }
  else if (myRobot.RDistance < rangeToAvoid){
    myRobot.turnLeft(collisionRotateSpeed);
  }
}
