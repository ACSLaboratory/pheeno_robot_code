/* This script makes Pheeno drive to different way points using only
encoder feedback!
*/

#include "Pheeno.h" // If you want to use Pheeno's premade routines import this!

Pheeno myRobot = Pheeno(0); // The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD)

float timeStep = 100; //Time step for the system to operate at (10Hz)!
int count = 0;//iterator through the way points.
float desVel = 10;//Desired linear velocity of Pheeno in cm/s.

//WayPoint Path Points.
float botYf[4]={0,0,75.0000,75.00}; //Pheeno's array of waypoint x positions.
float botXf[4]={0,75.0000,75.0000,0}; //Pheeno's array of waypoint y positions.

int numWayPoints = (sizeof(botXf)/sizeof(float));//Number of waypoints entered.

void setup(){// The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD)
  myRobot.SETUP(); //This must be included in every script! It sets up all the pins on the robot!
}

void loop() {
  while (calculateDistance(myRobot.botXPos,myRobot.botYPos,botXf[count%numWayPoints],botYf[count%numWayPoints]) > 3){
    myRobot.PIDWayPointControl(botXf[count%numWayPoints],botYf[count%numWayPoints],desVel,timeStep);
    myRobot.encoderPositionUpdate(timeStep); //Encoders used for state estimates.
  }
  count ++;
}

float calculateDistance(float x1, float y1, float x2, float y2){
  //Calculates the distance the robot is from the desired way point
  float space = sqrt(sq((x1 - x2)) + sq((y1 - y2)));
  return space;
}
