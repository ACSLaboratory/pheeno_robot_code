// Sean Wilson
// Arizona State University 2015

/*This script drives "Pheeno" to waypoints given through
Serial commands issued as x0,y0,a0;xf,yf,af. Initial position
and heading to final position and heading!
*/
#include "Pheeno.h"
#include "Encoder.h"
#include "Wire.h"
#include "LSM303.h"
#include "EEPROM.h"

String inData, state; // Communication Storage Things
boolean stringComplete = false;  // whether the string is complete

const float pi = 3.1415926; // PI...the number...duh!

float desVel = 15; //cm/s
float timeStep = 100; //ms

float botXf = 0;//Desired Positions
float botYf = 0;

// The setup() method runs once, when the sketch starts
void setup(){
  Serial.begin(9600); 
  Pheeno.SETUP();
}

void loop()                     
{
  serialEvent(); //Read the serial port!
  // Parse the incoming data.
  if (stringComplete) {
    parseData();
    // clear the string:
    inData = "";
    stringComplete = false;
  }
  if (calculateDistance(Pheeno.botXPos,Pheeno.botYPos,botXf,botYf) > 2){
    Pheeno.PIDWayPointControl(botXf,botYf,desVel,timeStep);
    Pheeno.encoderPositionUpdate(timeStep);
  }
  else{
    Pheeno.brake();
  }  
}
////////////////////////////////////////////////////////
///  BOT Sub Routines
////////////////////////////////////////////////////////

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inData += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '!') {
      stringComplete = true;
      Serial.println(inData);
    }
  }
}

void parseData() 
{
  state = inData.substring(0,1);
  if (state=="M"){ // -1 are needed to correct coordinate frames.
    Pheeno.botXPos = -1 * inData.substring(inData.indexOf('x')+1,inData.indexOf('y')).toInt();
    Pheeno.botYPos = inData.substring(inData.indexOf('y')+1,inData.indexOf('h')).toInt();
    Pheeno.botA = (inData.substring(inData.indexOf('h')+1,inData.indexOf('X')).toInt()) * pi/180;
    botXf = -1 * inData.substring(inData.indexOf('X')+1,inData.indexOf('Y')).toInt();
    botYf = inData.substring(inData.indexOf('Y')+1,inData.indexOf('!')).toInt();
  }
  
  if (state=="S"){
    botXf = Pheeno.botXPos;
    botYf = Pheeno.botYPos;
  }
}

float calculateDistance(float x1, float y1, float x2, float y2){
  //Calculates the distance the robot is from the leader
  float space = sqrt(sq((x1 - x2)) + sq((y1 - y2)));
  return space;
}
