#include <EEPROM.h>

int accX = 0;
int accY = 0;
int accZ = 0;

int magXMin = 0;
int magXMax = 0;
int magYMin = 0;
int magYMax = 0;
int magZMin = 0;
int magZMax = 0;

int eeAddress = 0;

void setup(){
  Serial.begin( 9600 );
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println( "Read accelerometer data from EEPROM: " );

  //Get the float data from the EEPROM at position 'eeAddress'
  EEPROM.get(eeAddress, accX);
  Serial.println(accX);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, accY);
  Serial.println(accY);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, accZ);
  Serial.println(accZ);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  Serial.println( "Read magnetometer data from EEPROM: " );
  
  EEPROM.get(eeAddress, magXMin);
  Serial.println(magXMin);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, magYMin);
  Serial.println(magYMin);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, magZMin);
  Serial.println(magZMin);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, magXMax);
  Serial.println(magXMax);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, magYMax);
  Serial.println(magYMax);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
  
  EEPROM.get(eeAddress, magZMax);
  Serial.println(magZMax);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  eeAddress += sizeof(int);
}

void loop(){ /* Empty loop */ }
