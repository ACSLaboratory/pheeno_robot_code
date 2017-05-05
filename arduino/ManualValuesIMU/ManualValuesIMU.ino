/*This script is used to calibrate the Pheeno robot's IMU. Instructions for use can be found
in the Pheeno Robot Programming Guide.*/

#include <Wire.h>
#include <LSM303.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

float accX = 0;
float accY = 0;
float accZ = 0;

float count = 0;

char report[80];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
}

void loop() {
  count = count + 1;  
  compass.read();
  
  //Compass Max and Min
  
  running_min.x = min(running_min.x, compass.m.x);
  running_min.y = min(running_min.y, compass.m.y);
  running_min.z = min(running_min.z, compass.m.z);

  running_max.x = max(running_max.x, compass.m.x);
  running_max.y = max(running_max.y, compass.m.y);
  running_max.z = max(running_max.z, compass.m.z);
  
  //Acceleration Mean
  
  accX = (count - 1)/count * accX + 1/count * compass.a.x;
  accY = (count - 1)/count * accY + 1/count * compass.a.y;
  accZ = (count - 1)/count * accZ + 1/count * compass.a.z;
  
  
  snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z);
  
  Serial.println("Magnetometer!");
  Serial.println(report);
  Serial.println("Accelerometer!");
  Serial.print(int(accX));
  Serial.print(", ");
  Serial.print(int(accY));
  Serial.print(", ");
  Serial.println(int(accZ));
  Serial.println("Orientation!");
  Serial.println(360-compass.heading());
  Serial.println();
  
}
