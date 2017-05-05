#ifndef Pheeno_h
#define Pheeno_h

#include "EEPROM.h"
#include "Encoder.h"
#include "LSM303.h"
#include "Wire.h"

class Pheeno
{
  public:
    Pheeno(int type);

    ///////////////////////////////////////////////////////////
    //IR Sensor Distance Variables
    ///////////////////////////////////////////////////////////

    float RDistance;
    float CDistance;
    float LDistance;
    float LFDistance;
    float RFDistance;
    float BDistance;

    ///////////////////////////////////////////////////////////
    //Encoder Counter Variables
    ///////////////////////////////////////////////////////////

    int encoderCountL;
    int encoderCountR;

    ///////////////////////////////////////////////////////////
    //IMU Variables (Magnetometer/Accelerometer)
    ///////////////////////////////////////////////////////////

    float IMUOrientation;
    float IMUACCX;
    float IMUACCY;
    float IMUACCZ;

    ///////////////////////////////////////////////////////////
    //Robot State Variables
    ///////////////////////////////////////////////////////////

    float botVel;
    float botXPos;
    float botYPos;
    float botA;

    ///////////////////////////////////////////////////////////
    //Constants
    ///////////////////////////////////////////////////////////

    int encoderCountsPerRotation; // Encoder counts per shaft rotation.
    float motorGearRatio; // The gearing ratio of the drive motor being used.
    float wheelDiameter; // Wheel Diameter in cm.
    float axelLength; // Axel length in cm.

    ///////////////////////////////////////////////////////////
    //Functions
    ///////////////////////////////////////////////////////////

    void SETUP(); //Setup Routine

    float deg2rad(float deg); //Converts degrees to radians.
    float rad2deg(float rad); //Converts radians to degrees.
    float wrapToPi(float rad); //Wraps a radian angle to (-pi,pi]
    float wrapTo2Pi(float rad); //Wrap a radian angle to [0,2*pi)

    float convertUnicycleToRightMotor(float vel, float w); // Unicycle Model Conversions
    float convertUnicycleToLeftMotor(float vel, float w); // Unicycle Model Conversions

    void readIR(); //Read The IR Sensors
    void readEncoders(); //Read The Encoders
    void readCompass(float magNorthOffset); //Read the Compass (in degrees)
    void readAccel(); //Read the Accelerometer (in cm/s^2)

    void encoderPositionUpdate(float timeStep); //Encoders used for state estimates.
    void sensorFusionPositionUpdate(float timeStep, float northOffset);//Complementary Fileter for Sensor Fusion State Estimates.

    void forwardL(int motorSpeed); //Rotate Left Motor Forward at Speed motorSpeed (0-255)
    void forwardR(int motorSpeed); //Rotate Right Motor Forward at Speed motorSpeed (0-255)
    void reverseL(int motorSpeed); //Rotate Left Motor Backwards at Speed motorSpeed (0-255)
    void reverseR(int motorSpeed); //Rotate Right Motor Backwards at Speed motorSpeed (0-255)

    void forward(int motorSpeed);//Rotate left and right motor forward at the same speed motorSpeed (same direction)
    void reverse(int motorSpeed);//Rotate left and right motor backwards at the same speed motorSpeed (same direction)
    void turnLeft(int motorSpeed);//Rotate  left motor backwards and right motor forwards at the same speed motorSpeed
    void turnRight(int motorSpeed);//Rotate  left motor forwards and right motor backwards at the same speed motorSpeed

    void brake();//Short the Motors to Brake (No Passive Rotation)
    void noMotion();//Turn off motors (Allow Passive Rotation)

    void rotateAboutICC(float R, float WSpeed);//Rotate about an Instantaneous Center of Curvature (ICC) of radius R (cm) at angular speed WSpeed (rad/s).
    void PIDMotorControl(float desLVel, float desRVel); //PID Controller to Keep Wheels Rotating at Proper Speed
    void PIDWayPointControl(float u1, float u2, float desVelocity, float timeStep); //PID Controller (on the heading) to move Pheeno to a Way Point with coordinates (u1,u2)

  private:
    int _type; // The type of Pheeno being used! (0 = DD, 1 = Tank Tread, 2 = Tripod DD)

    ///////////////////////////////////////////////////////////
    //Constants
    ///////////////////////////////////////////////////////////
    const float pi; // Pi...the number...DUH!!

    ///////////////////////////////////////////////////////////
    //Pin Numbers Here
    ///////////////////////////////////////////////////////////

    const uint8_t IRC; // 0 // Analog input center sensor
    const uint8_t IRLF;// 1 // Analog input left forward sensor
    const uint8_t IRL;// 2 // Analog input left sensor
    const uint8_t IRB;// 3 // Analog input back sensor
    const uint8_t IRRF;// 6 // Analog input right forward sensor
    const uint8_t IRR;// 7 // Analog input right sensor

    const uint8_t PWMA;// 11 // A Motor PWM Control
    const uint8_t PWMB;// 5 // B Motor PWM Control

    const uint8_t AMotor1;// 9 // A Motor Direction 1
    const uint8_t AMotor2;// 10 // A Motor Direction 2

    const uint8_t BMotor1;// 7 // B Motor Direction 1
    const uint8_t BMotor2;// 6 // B Motor Direction 2

    const uint8_t STBY;// 8 // Standby pin to turn off motors.

    const uint8_t interruptL;// 3 // Left motor encoder interrupt
    const uint8_t interruptR;// 2 // Right motor encoder interrupt

    const uint8_t noInterruptL;// 12 // Left motor encoder no interrupt
    const uint8_t noInterruptR;// 13 // Right motor encoder no interrupt

    ///////////////////////////////////////////////////////////
    //IR Distance Conversion Constants
    ///////////////////////////////////////////////////////////

    //Constants from distance conversion see getDistance()

    const double a;// 15.68
    const double b;// -0.03907

    ///////////////////////////////////////////////////////////
    //Motor PID Control Constants
    ///////////////////////////////////////////////////////////

    float PIDMotorsTimeStart;

    float kpMotor;// = 0.904;
    float kiMotor;// = 146;
    int kdMotor;// = 0;

    float motorDigitalK;// = 0.544;

    ///////////////////////////////////////////////////////////
    //Rotational PID Control Constants
    ///////////////////////////////////////////////////////////

    // Rotational Controller Gains!
    float kpAng;// = 0.932;
    float kiAng;// = 0.548;
    float kdAng;// = 0;

    float rotationalDigitalK;// = 2.5;

    ///////////////////////////////////////////////////////////
    //PID Way Point Control Constants
    ///////////////////////////////////////////////////////////

    float PIDWayPointTimeStart;

    // Error for PIDWayPointControl
    float errorAng;
    float integralAng;
    float diffAng;
    float oldErrorAng;

    //Control Variable for PIDWayPointControl
    float PIDWayPointW;

    ///////////////////////////////////////////////////////////
    //IMU Things (Compass/Accelerometer)
    ///////////////////////////////////////////////////////////
    LSM303 compass;
    LSM303::vector<int16_t> running_min, running_max;
    int eeAddress;// = 0;

    int accXOff;// = 0;
    int accYOff;// = 0;
    int accZOff;// = 0;

    //Storage for last heading of robot used in sensorFusionPositionUpdate
    float botA0;// = 0;

    ///////////////////////////////////////////////////////////
    //Encoder Things
    ///////////////////////////////////////////////////////////

    Encoder motorLeft, motorRight;

    int motorL;//Left Motor Speed (Arduino PWM Units, int 0-255)
    int motorR;//Right Motor Speed (Arduino PWM Units, int 0-255)

    float oldErrorL; //old speed error of motor for PIDMotorControl Function
    float oldErrorR;

    int oldMotorPIDEncoderCountL;//Old Encoder Count storage for PIDMotorControl Function
    int oldMotorPIDEncoderCountR;

    int oldEPUEncoderCountL;//Old Encoder Count storage for encoderPositionUpdate Function
    int oldEPUEncoderCountR;

    int oldSFPUEncoderCountL;//Old Encoder Count storage for sensorFusionPositionUpdate Function
    int oldSFPUEncoderCountR;

    ///////////////////////////////////////////////////////////
    //Position Update Timer
    ///////////////////////////////////////////////////////////

    float positionUpdateTimeStart;



};

#endif
