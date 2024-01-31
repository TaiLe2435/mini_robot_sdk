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

Estimation est;

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

//__________ CODE TO TEST ESTIMATION CLASS ___________//

  VectorXd estimation{6};
  Vector3d foo;
  VectorXd temp{6};

  temp = get_pose();

//   foo << 0,0,0;
//   temp << 0,0,0,0,0,0;

//   estimation = est.imu_ekf(foo, foo, foo, foo, temp, foo);

  Serial.println("Pose:");
//  Print the components of an eigen struct
    for (int i = 0; i < temp.rows(); ++i) {
        for (int j = 0; j < temp.cols(); ++j) {
            Serial.print(temp(i,j));
            Serial.print(" ");
        }
        Serial.println("");
    }

// ___________________ CODE FOR TESTING BT __________//
  // Serial.println("This: ");
  // byte* desired = getBT();
  // for (int i=0; i<8; i++){
  //   Serial.print(desired[i]);
  //   Serial.print(", ");
  // }
  // Serial.println("");
  // Serial.println("Ints: ");
  // for (int j=0; j<5; j+=2){
  //   Serial.print(desired[j] + desired[j+1]*256);
  //   Serial.print(", ");
  // }
  // Serial.println("");

  //_______________________POTENTIAL MAIN LOOP ___________//
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

