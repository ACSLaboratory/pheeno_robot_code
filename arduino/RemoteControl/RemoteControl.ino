// Receives Serial Command from RPi and triggers motors appropriately.
// Sean Wilson
// Arizona State University 2016

/*Takes commands from the Raspberry Pi and converts them 
to motion!*/

#include "Pheeno.h"

Pheeno myRobot = Pheeno(1);

// Buffer to store incoming commands from serial port
String inData, str;
int val;
boolean stringComplete = false;  // whether the string is complete


void setup() {
   Serial.begin(9600);
   myRobot.SETUP();    
}

void loop()
{
  serialEvent(); //Read the serial port!
  // Parse the incoming data.
  if (stringComplete) {
    parseData(); //Get the useful information from the string
    // clear the string:
    inData = "";
    stringComplete = false;
  }
   runAround(); //Do actions based on the input.
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
    if (inChar == ':') {
      stringComplete = true;
      Serial.println(inData);
    }
  }
}

void parseData() 
{
   if (inData.indexOf(':')>=0)
   {
     str = inData.substring(0,inData.indexOf(';'));
     val = inData.substring(inData.indexOf(';')+1,inData.indexOf(':')).toInt();
   }
   else
   {
     str = inData;
     val = 0;
   }
}

void runAround(){   
  
  if (str == "F"){
    myRobot.forward(val);
  }
  
  else if (str == "B"){
    myRobot.reverse(val);
  }
  
  else if (str == "L"){
    myRobot.turnLeft(val);
  }
  
  else if (str == "R"){
    myRobot.turnRight(val);
  }
  
  else if (str == "Z" || inData == ""){
    myRobot.brake();
  }
  
  else {
    myRobot.brake();
  }
  
}
