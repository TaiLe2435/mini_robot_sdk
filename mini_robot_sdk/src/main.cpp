#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

// Header files
#include "IMU.h"
#include "motion.h"
#include "bluetooth.h"
#include "estimation.h"

// Estimation imu;
// VectorXd calibration{6};

float* imu_calibration;
float* heading;

void setup() {
  stop();

  Serial.begin(115200);
  Serial.println("setting up :)");

  // calibration = imu.calibrate_imu();
  // for(int m=0; m<calibration.rows(); m++){
  //   Serial.println(calibration(m));
  // }
  // Serial.println("");
  // delay(1000);

  // Vector3d gyro_calib;
  // gyro_calib << calibration(0), calibration(1), calibration(2);
  // Vector3d gyro;
  // gyro = imu.get_gyro(gyro_calib);
  // for(int m=0; m<gyro.rows(); m++){
  //   Serial.println(gyro(m));
  // }
  // Serial.println("");
  // delay(1000);

  // Vector3d acc;
  // acc = imu.get_acc();
  // for(int n=0; n<acc.rows(); n++){
  //   Serial.println(acc(n));
  // }
  // Serial.println("");
  // delay(1000);

  initIMU();

  BTinit();
  Serial.println("Entering loop ... <(o.o)>");
  delay(1000);
}

void loop() {
  stop();

  heading = poseEstimation();
  Serial.print("Angles: ");
  Serial.print(heading[0]);
  Serial.print(" ");
  Serial.print(heading[1]);
  Serial.print(" ");
  Serial.println(heading[2]);

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
//   Eigen::Vector3d rpy_error;
//   rpy_error << 0, 0, 0;
//   Eigen::Vector3d states = imu.get_rpy(calibration, rpy_error);

//  // Print the elements of the 3x3 matrix
//     for (int i = 0; i < states.rows(); ++i) {
//         for (int j = 0; j < states.cols(); ++j) {
//             Serial.print(states(i, j));
//             Serial.print(" ");
//         }
//         Serial.println("");
//     }

  // Serial.println("Gyro data:");
  // Serial.println(states[0]);
  // Serial.println(states[1]);
  // Serial.println(states[2]);
  delay(100);

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

