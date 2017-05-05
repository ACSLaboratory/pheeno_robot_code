/*This script is used to calibrate the Pheeno robot's IMU. Stores the values in the 
EEPROM.*/

#include <LSM303.h>
#include <EEPROM.h>

LSM303 compass;
//Input Compass Values Here!
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

//Input Accelerometer Values Here!
int accX = 0;
int accY = 0;
int accZ = 0;

int eeAddress = 0;

void setup() {
  //Store Values to the EEPROM
  EEPROM.put(eeAddress, int(accX));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(accY));
  eeAddress += sizeof(int);
  EEPROM.put(eeAddress, int(accZ));
  eeAddress += sizeof(int);
  
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
}

void loop() { 
}
