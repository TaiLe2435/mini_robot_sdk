#include <Arduino.h>

// Header files
#include "IMU/IMU.h"
#include "motion/motion.h"
#include "bluetooth/bluetooth.h"

float gyroCalib[3] {0};

void setup() {
  Serial.begin(115200);
  Serial.println("setting up :)");

  float* calib = initIMU();
  gyroCalib[0] = calib[0];
  gyroCalib[1] = calib[1];
  gyroCalib[2] = calib[2];

  BTinit();
}

void loop() {
  Serial.println("looping ... <(o.o)>");

  int desired = getBT();

  float heading = poseEstimation(gyroCalib[0], gyroCalib[1], gyroCalib[2]);

  float kP {0.5};
  if(desired != 9999)
  {
    float error = desired - (int)heading;
    float leftSpeed = 200 + kP*error;
    float rightSpeed = 200 - kP*error;
    move_wheels(leftSpeed, rightSpeed);
  }
  else if (desired == 9999)
  {
    stop();
  }
  else
  {
    stop();
    Serial.println("Error in bluetooth data: unknown value received");
    while(1);
  }
}

