#include "Arduino.h"
#include "Pheeno.h"

Pheeno::Pheeno(int type):pi(3.1415926),IRC(0),IRLF(1),IRL(2),IRB(3),IRRF(6),IRR(7), \
PWMA(11),PWMB(5),AMotor1(9),AMotor2(10),BMotor1(7),BMotor2(6),STBY(8),interruptL(3), \
interruptR(2),noInterruptL(12),noInterruptR(13),a(15.68),b(-0.03907),motorRight(interruptR, noInterruptR),\
motorLeft(interruptL, noInterruptL){
  ///////////////////////////////////////////////////////////
  //Set kinematic measurements and PID Gains based on base type.
  ///////////////////////////////////////////////////////////
  encoderCountsPerRotation = 12;

  // The argument is the type of robot (0 = Original DD, 1 = Tank Treads, 2 = Tripod DD, 3 = Tripod 100:1 Gear Motor)
  if (type == 0){
    motorGearRatio = 51.45; // The gearing ratio of the drive motor being used.
    wheelDiameter = 3.2; // Wheel Diameter in cm.
    axelLength = 10.116; // Axel length in cm.

    //Motor PID Control Constants
    kpMotor = 0.904;
    kiMotor = 146;
    kdMotor = 0;
    motorDigitalK = 0.544;

    // Rotational Controller Gains!
    kpAng = 0.932; 
    kiAng = 0.548;
    kdAng = 0;
    rotationalDigitalK = 2.5;

  }
  else if (type == 1){
    motorGearRatio = 100.37; // The gearing ratio of the drive motor being used.
    wheelDiameter = 3.9; // Wheel Diameter in cm.
    axelLength = 11.9; // Axel length in cm.

    //Motor PID Control Constants
    kpMotor = 4.336;
    kiMotor = 304.066;
    kdMotor = 0;
    motorDigitalK = 0.564;

    // Rotational Controller Gains!
    kpAng = 0.932; 
    kiAng = 0.548;
    kdAng = 0;
    rotationalDigitalK = 5;    

  }
  else if (type == 2){
    motorGearRatio = 51.45; // The gearing ratio of the drive motor being used.
    wheelDiameter = 3.2; // Wheel Diameter in cm.
    axelLength = 13.351; // Axel length in cm.

    //Motor PID Control Constants
    kpMotor = 0.904;
    kiMotor = 146;
    kdMotor = 0;
    motorDigitalK = 0.544;

    // Rotational Controller Gains!
    kpAng = 0.932; 
    kiAng = 0.548;
    kdAng = 0;
    rotationalDigitalK = 2.5;    

  }
  else if (type == 3){
    motorGearRatio = 100.37; // The gearing ratio of the drive motor being used.
    wheelDiameter = 3.2; // Wheel Diameter in cm.
    axelLength = 14.0; // Axel length in cm.

    //Motor PID Control Constants
    kpMotor = 4.01;
    kiMotor = 125.82;
    kdMotor = -0.0481;
    motorDigitalK = 0.83;

    // Rotational Controller Gains!
    kpAng = 0.932; 
    kiAng = 0.548;
    kdAng = 0;
    rotationalDigitalK = 2.5;    

  }
  _type = type;
}

///////////////////////////////////////////////////////////
//Setup
///////////////////////////////////////////////////////////

void Pheeno::SETUP(){
  Wire.begin();
  compass.init();
  compass.enableDefault();

  pinMode(interruptL , INPUT);
  pinMode(interruptR , INPUT);
  
  pinMode(noInterruptL , INPUT);
  pinMode(noInterruptR , INPUT);
  
  pinMode(PWMA, OUTPUT); 
  pinMode(PWMB, OUTPUT);  
  
  pinMode(AMotor1, OUTPUT); 
  pinMode(AMotor2, OUTPUT); 
  
  pinMode(BMotor1, OUTPUT); 
  pinMode(BMotor2, OUTPUT);
  
  pinMode(STBY, OUTPUT);

  //Get the float data from the EEPROM at position 'eeAddress'
  EEPROM.get(eeAddress, accXOff);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, accYOff);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, accZOff);
  eeAddress += sizeof(int);

  EEPROM.get(eeAddress, running_min.x);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, running_min.y);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, running_min.z);
  eeAddress += sizeof(int);

  EEPROM.get(eeAddress, running_max.x);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, running_max.y);
  eeAddress += sizeof(int);
  EEPROM.get(eeAddress, running_max.z);

  // Compass Calibration Data (Max and Min Vectors)
  compass.m_min = (LSM303::vector<int16_t>){ running_min.x,  running_min.y,  running_min.z};
  compass.m_max = (LSM303::vector<int16_t>){ running_max.x,  running_max.y,  running_max.z};

  

  PIDMotorsTimeStart = millis();
  PIDWayPointTimeStart = millis();
  positionUpdateTimeStart = millis();
}

///////////////////////////////////////////////////////////
//Conversion Functions
///////////////////////////////////////////////////////////

float Pheeno::deg2rad(float deg){
  //Converts degree to radians
  float output = deg * pi / 180;
  return output;
}

float Pheeno::rad2deg(float rad){
  //Converts radians to degree
  float output = rad * 180 / pi;
  return output;
}

float Pheeno::wrapToPi(float rad){
  //Converts radian angle (-pi,pi]
  float output = atan2(sin(rad),cos(rad));
  return output;
}

float Pheeno::wrapTo2Pi(float rad){
  //Converts radian angle [0,2*pi)
  float output = wrapToPi(rad);
  if (output < 0){
    output = output + 2*pi;
  }
  return output;
}

///////////////////////////////////////////////////////////
// Unicycle Model Conversions
///////////////////////////////////////////////////////////

float Pheeno::convertUnicycleToRightMotor(float vel, float w){
  float output = (2.0*vel+w*axelLength)/(wheelDiameter);
  return output;
}

float Pheeno::convertUnicycleToLeftMotor(float vel, float w){
  float output = (2.0*vel-w*axelLength)/(wheelDiameter);
  return output;
}

///////////////////////////////////////////////////////////
//Read Sensors
///////////////////////////////////////////////////////////

void Pheeno::readIR(){ // Gets a distance from IR sensors in cm.
  // Takes the value from the IR and converts it to a voltage.
  
  float voltsR = analogRead(IRR)* 0.00322265625;   // value from right sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsC = analogRead(IRC)* 0.00322265625;   // value from center sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsL = analogRead(IRL)* 0.00322265625;   // value from left sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsLF = analogRead(IRLF)* 0.00322265625;   // value from left forward sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsRF = analogRead(IRRF)* 0.00322265625;   // value from right forward sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float voltsB = analogRead(IRB)* 0.00322265625;   // value from right forward sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  
  
  /* This calibration is for the Sharp GP2Y0A41SK0F IR distance sensor.(http://www.pololu.com/product/2464)
  
  This measures to the back of the sensor! If you want the distance to the lense subtract off the width of the sensor.
    The units are in CM.
  
  This sensor has good accuracy from 4~30 cm then starts to get fuzzy.
  
  D=a/(V-b);
  
  a=15.68;
  b=-0.03907;
 
  Width of the Sensor=1.3;
  */
     RDistance = a/(voltsR - b);
     CDistance = a/(voltsC - b);
     LDistance = a/(voltsL - b);
     LFDistance = a/(voltsLF - b);
     RFDistance = a/(voltsRF - b);
     BDistance = a/(voltsB - b);
}

void Pheeno::readEncoders(){
  encoderCountR = motorRight.read();
  encoderCountL = -motorLeft.read();
}

void Pheeno::readCompass(float magNorthOffset){
  compass.read();
  /* Here the deg2rad(360-compass.heading()-magNorthOffset) is due to the fact the compass puts out
  it's heading in degrees with increasing angle being clockwise. The offset is the difference between
  the global frame's defined x-axis (0 degrees) and magnetic north (CCW Positive).*/
  IMUOrientation = (360.0 - float(compass.heading()) - magNorthOffset);  
}

void Pheeno::readAccel(){
  compass.read();
  IMUACCX = (compass.a.x - accXOff) * 0.061 / 100 * 9.81;
  IMUACCY = (compass.a.y - accYOff) * 0.061 / 100 * 9.81;
  IMUACCZ = (compass.a.y - accZOff) * 0.061 / 100 * 9.81;
}

void Pheeno::encoderPositionUpdate(float timeStep){

  if (millis() - positionUpdateTimeStart >= timeStep){
    timeStep = (millis() - positionUpdateTimeStart)/1000; //Convert ms to s
    positionUpdateTimeStart = millis();

    readEncoders();
    int countL = encoderCountL;
    int countR = encoderCountR;

    float Dl = pi*wheelDiameter * (countL - oldEPUEncoderCountL) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    float Dr = pi*wheelDiameter * (countR - oldEPUEncoderCountR) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    //Check integer roll over!
    if (countL < 0 && oldEPUEncoderCountL > 0){
      Dl = pi*wheelDiameter * ((countL - (-32768)) + (32767 - oldEPUEncoderCountL)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    if (countR < 0 && oldEPUEncoderCountR > 0){
      Dr = pi*wheelDiameter * ((countR - (-32768)) + (32767 - oldEPUEncoderCountR)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    float Dc = (Dr + Dl)/2; //Distance center of bot has moved read by encoders.
    oldEPUEncoderCountR = countR;
    oldEPUEncoderCountL = countL;
    
    botA += (Dr - Dl)/axelLength;

    botXPos += Dc * cos(botA0 + (botA-botA0)/2);
    botYPos += Dc * sin(botA0 + (botA-botA0)/2);

    botVel = (Dr + Dl)/(2*timeStep);
    botA0 = botA;
  }  
}

void Pheeno::sensorFusionPositionUpdate(float timeStep, float northOffset){

  if (millis() - positionUpdateTimeStart >= timeStep){
    timeStep = (millis() - positionUpdateTimeStart)/1000; //Convert ms to s
    positionUpdateTimeStart = millis();

    readEncoders();
    int countL = encoderCountL;
    int countR = encoderCountR;

    float Dr = pi * wheelDiameter * (countR - oldSFPUEncoderCountR) / (encoderCountsPerRotation * motorGearRatio); // Linear distance right wheel has rotated.
    float Dl = pi * wheelDiameter * (countL - oldSFPUEncoderCountL) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    //Check integer roll over!
    if (countL < 0 && oldSFPUEncoderCountL > 0){
      Dl = pi*wheelDiameter * ((countL - (-32768)) + (32767 - oldSFPUEncoderCountL)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    if (countR < 0 && oldEPUEncoderCountR > 0){
      Dr = pi*wheelDiameter * ((countR - (-32768)) + (32767 - oldSFPUEncoderCountR)) / (encoderCountsPerRotation * motorGearRatio); // Linear distance left wheel has rotated.
    }
    float Dc = (Dr + Dl)/2; //Distance center of bot has moved read by encoders.
    oldSFPUEncoderCountR = countR;
    oldSFPUEncoderCountL = countL;

    float botD = 0.8 * Dc + 0.2 * botVel * timeStep;
    
    readCompass(northOffset);
    if (botA0 > -pi/2 && botA0 < pi/2){
      botA = 0.9 * wrapToPi(botA + (Dr - Dl)/axelLength) + 0.1 * wrapToPi(deg2rad(IMUOrientation));
    }
    else{
      botA = 0.9 * wrapTo2Pi(botA + (Dr - Dl)/axelLength) + 0.1 * wrapTo2Pi(deg2rad(IMUOrientation));
    }

    botXPos += botD * cos(botA0 + (botA-botA0)/2);
    botYPos += botD * sin(botA0 + (botA-botA0)/2);

    readAccel();
    botVel = 0.95 * Dc/timeStep + 0.05 * (botVel + IMUACCY * timeStep);
    botA0 = botA;
  }  
}

///////////////////////////////////////////////////////////
//Motor Functions
///////////////////////////////////////////////////////////

void Pheeno::noMotion(){
  // Turns the motors off. (They can still rotate passively) 
  digitalWrite(STBY, LOW); //Motors OFF
}

void Pheeno::brake(){
  //Brakes the motors. (Locked to not rotate passively, this can be overcome by enough torque)
  digitalWrite(STBY, HIGH);
  
  analogWrite(PWMA, LOW);
  digitalWrite(AMotor1, HIGH);
  digitalWrite(AMotor2, HIGH);
  
  analogWrite(PWMB, LOW);
  digitalWrite(BMotor1, HIGH);
  digitalWrite(BMotor2, HIGH);
}

void Pheeno::forwardL(int motorSpeed){
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

void Pheeno::forwardR(int motorSpeed){
  // Right motor drive forward.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  } 
  digitalWrite(STBY, HIGH); // Motors ON 
  analogWrite(PWMB, motorSpeed); // Speed
  digitalWrite(BMotor1, LOW);    
  digitalWrite(BMotor2, HIGH); 
}

void Pheeno::reverseL(int motorSpeed){
  // Left motor drive backwards.
  if (motorSpeed>255){
    motorSpeed=255;
  }
  if (motorSpeed<0){
    motorSpeed=0;
  } 
  digitalWrite(STBY, HIGH); // Motors ON 
  analogWrite(PWMA, motorSpeed); // Speed
  digitalWrite(AMotor1, HIGH);    
  digitalWrite(AMotor2, LOW);  
}

void Pheeno::reverseR(int motorSpeed){
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

void Pheeno::forward(int motorSpeed){
  // Both motors drive forwards.
  forwardL(motorSpeed);
  forwardR(motorSpeed);  
}

void Pheeno::reverse(int motorSpeed){
  // Both motors drive backwards.
  reverseL(motorSpeed);
  reverseR(motorSpeed);  
}

void Pheeno::turnLeft(int motorSpeed){
  reverseL(motorSpeed);
  forwardR(motorSpeed);  
}

void Pheeno::turnRight(int motorSpeed){
  reverseR(motorSpeed);
  forwardL(motorSpeed);  
}

///////////////////////////////////////////////////////////
//Controllers
///////////////////////////////////////////////////////////

void Pheeno::PIDMotorControl(float desLVel, float desRVel){
    /*Keeps the rotational speeds of the individual motors at setpoints desLVel and desRVel (rad/s).*/

    float timeStep = 50;

    if (millis() - PIDMotorsTimeStart >= timeStep){
      float PIDTimeStep = (millis() - PIDMotorsTimeStart)/1000;//Time step for controller to work on (s).

      readEncoders();
      int countL = encoderCountL;
      int countR = encoderCountR;

      // Error on individual motors for vel control
      float errorL = desLVel - 2 * pi * (countL - oldMotorPIDEncoderCountL) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      float errorR = desRVel - 2 * pi * (countR - oldMotorPIDEncoderCountR) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      float integralL = integralL + errorL * PIDTimeStep;
      float integralR = integralR + errorR * PIDTimeStep;
      float diffL = (oldErrorL - errorL) / PIDTimeStep;
      float diffR = (oldErrorR - errorR) / PIDTimeStep;
      oldErrorL = errorL;
      oldErrorR = errorR;
      oldMotorPIDEncoderCountL = countL;
      oldMotorPIDEncoderCountR = countR;

      motorL += int(motorDigitalK*(kpMotor*errorL + kiMotor*integralL + kdMotor*diffL));
      motorR += int(motorDigitalK*(kpMotor*errorR + kiMotor*integralR + kdMotor*diffR));

      if (motorL>255){
        motorL=255;
      }
      if (motorR>255){
        motorR=255;
      }
      if (motorL<-255){
        motorL=-255;
      }
      if (motorR<-255){
        motorR=-255;
      }
      if (motorL >= 0){
        forwardL(motorL);
      }
      if (motorR >= 0){
        forwardR(motorR);
      }
      if (motorL < 0){
        reverseL(abs(motorL));
      }
      if (motorR < 0){
        reverseR(abs(motorR));
      }
      PIDMotorsTimeStart = millis();
    }
}

void Pheeno::PIDWayPointControl(float u1, float u2, float desVelocity, float timeStep){
  /*Causes Pheeno to move to waypoint located at (u1,u2) assuming inial
  condition X0,Y0,A0 defined initially and updated by the robot's
  odometry.*/

  if (millis() - PIDWayPointTimeStart >= timeStep){
    float PIDTimeStep = (millis()-PIDWayPointTimeStart)/1000;
    
    float desHeading = atan2((u2-botYPos), (u1-botXPos));// rad
    
    //Calculate Errors for PID control
    errorAng = wrapToPi(desHeading - botA); 
    integralAng = wrapToPi(integralAng + errorAng * PIDTimeStep);
    diffAng = wrapToPi((oldErrorAng - errorAng) / PIDTimeStep);
    oldErrorAng = errorAng;  
     
    PIDWayPointW = rotationalDigitalK*(kpAng*errorAng + kiAng*integralAng + kdAng*diffAng);
      
    PIDWayPointTimeStart = millis(); 
  }
  // motorLVel and motorRVel are in rad/s
  float motorLVel = convertUnicycleToLeftMotor(desVelocity, PIDWayPointW);
  float motorRVel = convertUnicycleToRightMotor(desVelocity, PIDWayPointW);

  PIDMotorControl(motorLVel, motorRVel);
}

void Pheeno::rotateAboutICC(float R, float WSpeed){
  /*Causes Pheeno to rotate about an Instantaneous Center of 
  Curvature (ICC) of radius R (cm) at angular speed WSpeed (rad/s).*/

  float motorRVel = WSpeed*(R + axelLength/2.0);
  float motorLVel = WSpeed*(R - axelLength/2.0);
    
  PIDMotorControl(motorLVel, motorRVel);
}
