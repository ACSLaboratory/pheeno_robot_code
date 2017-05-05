/*This script is used to calibrate the Pheeno robot's IMU. Stores the values in the 
EEPROM.*/

#include <Wire.h>
#include <LSM303.h>
#include <EEPROM.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

float accX = 0;
float accY = 0;
float accZ = 0;

float count = 1;
int eeAddress = 0;

///////////////////////////////////////////////////////////
//Pin Numbers Here
///////////////////////////////////////////////////////////

const uint8_t IRC = 0; // Analog input center sensor
const uint8_t IRLF = 1; // Analog input left forward sensor
const uint8_t IRL = 2; // Analog input left sensor
const uint8_t IRB = 3; // Analog input back sensor
const uint8_t IRRF = 6; // Analog input right forward sensor
const uint8_t IRR = 7; // Analog input right sensor

const uint8_t PWMA = 11; // A Motor PWM Control
const uint8_t PWMB = 5; // B Motor PWM Control

const uint8_t AMotor1 = 9; // A Motor Direction 1
const uint8_t AMotor2 = 10; // A Motor Direction 2

const uint8_t BMotor1 = 7; // B Motor Direction 1
const uint8_t BMotor2 = 6; // B Motor Direction 2

const uint8_t STBY = 8; // Standby pin to turn off motors.

const uint8_t interruptL = 3;
const uint8_t interruptR = 2;

const uint8_t noInterruptL = 12;
const uint8_t noInterruptR = 13;

void setup() {
  Wire.begin();
  compass.init();
  compass.enableDefault();
}

void loop() {

  for(int i = 0; i<1000; i++){
      compass.read();
      
      //Acceleration Mean 
      accX = (count - 1)/count * accX + 1/count * compass.a.x;
      accY = (count - 1)/count * accY + 1/count * compass.a.y;
      accZ = (count - 1)/count * accZ + 1/count * compass.a.z;
      
      count = count + 1;  
  }
  
  //Store Values to the EEPROM
  EEPROM.put(eeAddress, int(accX));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(accY));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(accZ));
  eeAddress += sizeof(int);
  
  turnRight(100);  
  for(int j = 0; j<5000; j++){
    compass.read();
    
    //Compass Max and Min  
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);
  
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
  }
  
  //Store Values to the EEPROM
  EEPROM.put(eeAddress, running_min.x);
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, running_min.y);
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, running_min.z);
  eeAddress += sizeof(int);
  
  //Store Values to the EEPROM
  EEPROM.put(eeAddress, running_max.x);
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, running_max.y);
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, running_max.z);
  eeAddress += sizeof(int);
 
  while (true){
    noMotion();
  } 
}

void noMotion(){
  // Turns the motors off. (They can still rotate passively) 
  digitalWrite(STBY, LOW); //Motors OFF
}

void forwardL(int motorSpeed){
  // Left motor drive forward.  
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  }
  digitalWrite(STBY, HIGH); // Motors ON 
  analogWrite(PWMA, motorSpeed); // Speed
  digitalWrite(AMotor1, LOW);    
  digitalWrite(AMotor2, HIGH); 
}

void reverseR(int motorSpeed){
  // Right motor drive backwards.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  }
  digitalWrite(STBY, HIGH); // Motors ON 
  analogWrite(PWMB, motorSpeed); // Speed
  digitalWrite(BMotor1, HIGH);    
  digitalWrite(BMotor2, LOW);   
}

void turnRight(int motorSpeed){
  reverseR(motorSpeed);
  forwardL(motorSpeed);  
}
