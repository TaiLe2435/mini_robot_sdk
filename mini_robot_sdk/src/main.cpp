#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

// Header files
#include "IMU.h"
#include "motion.h"
#include "bluetooth.h"
#include "estimation.h"

Estimation imu;
float* calib {};

void setup() {
  stop();

  Serial.begin(115200);
  Serial.println("setting up :)");

  calib = imu.calib_gyro();
  delay(1000);

  BTinit();
  Serial.println("Entering loop ... <(o.o)>");
  delay(1000);
}

void loop() {
  stop();

  // int desired = getBT();

//   Eigen::MatrixXd states {12,12};
//   states = imu.imu_measurement_model(calib);

//  // Print the elements of the 12x12 matrix
//     for (int i = 0; i < states.rows(); ++i) {
//         for (int j = 0; j < states.cols(); ++j) {
//             Serial.print(states(i,j));
//             Serial.print(" ");
//         }
//         Serial.println("");
//     }

  // Eigen::Matrix3d states = imu.ddr_measurement_model();
  Eigen::Vector3d states = imu.ddr_measurement(calib);

 // Print the elements of the 3x3 matrix
    for (int i = 0; i < states.rows(); ++i) {
        for (int j = 0; j < states.cols(); ++j) {
            Serial.print(states(i, j));
            Serial.print(" ");
        }
        Serial.println("");
    }

  // Serial.println("Gyro data:");
  // Serial.println(states[0]);
  // Serial.println(states[1]);
  // Serial.println(states[2]);
  delay(5000);

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

  return;
}

