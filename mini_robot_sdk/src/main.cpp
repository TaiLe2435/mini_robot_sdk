#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

// Header files
#include "IMU.h"
#include "motion.h"
#include "bluetooth.h"
#include "estimation.h"

float* imu_calibration;
float* heading;

using namespace Eigen;

void setup() {
  stop();

  Serial.begin(115200);
  Serial.println("setting up :)");

  initIMU();

  BTinit();
  Serial.println("Entering loop ... <(o.o)>");
  delay(1000);
}

void loop() {
  stop();

  VectorXd estimation{6};

  estimation = get_pose();

  Serial.println("Pose:");
//  Print the components of an eigen struct
    for (int i = 0; i < estimation.rows(); ++i) {
        for (int j = 0; j < estimation.cols(); ++j) {
            Serial.print(estimation(i,j));
            Serial.print(" ");
        }
        Serial.println("");
    }

  // int desired = getBT();
  // float kP {0.5};
  // if(desired != 9999)
  // {
  //   float error = desired - (int)heading;
  //   float leftSpeed = 200 + kP*error;
  //   float rightSpeed = 200 - kP*error;
  //   move_wheels(leftSpeed, rightSpeed);
  // }
  // else if (desired == 9999)
  // {
  //   stop();
  // }
  // else
  // {
  //   stop();
  //   Serial.println("Error in bluetooth data: unknown value received");
  //   while(1);
  // }

  delay(100);

  return;
}

