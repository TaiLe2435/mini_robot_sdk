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
  // stop();

//__________ CODE TO TEST ESTIMATION CLASS ___________//

//   VectorXd estimation{6};
//   Vector3d foo;
//   VectorXd temp{6};

//   temp = get_pose();

// //   foo << 0,0,0;
// //   temp << 0,0,0,0,0,0;

// //   estimation = est.imu_ekf(foo, foo, foo, foo, temp, foo);

//   Serial.println("Pose:");
// //  Print the components of an eigen struct
//     for (int i = 0; i < temp.rows(); ++i) {
//         for (int j = 0; j < temp.cols(); ++j) {
//             Serial.print(temp(i,j));
//             Serial.print(" ");
//         }
//         Serial.println("");
//     }

// ___________________ CODE FOR TESTING BT __________//
  
  byte* desired = getBT();
  int x = desired[0] + desired[1] * 256;
  int y = desired[2] + desired[3] * 256;
  int theta = desired[4] + desired[5] * 256;

// Serial.println("This: ");
  for (int i=0; i<8; i++){
  Serial.print(desired[i]);
  Serial.print(", ");
  }
  Serial.println("");

  Serial.println("Ints: ");
  for (int j=0; j<5; j+=2){
    Serial.print(desired[j] + desired[j+1]*256);
    Serial.print(", ");
  }
  Serial.println("");

  //_______________________POTENTIAL MAIN LOOP ___________//
  float kP {0.5};
  if (x == 0 && y == 0 && theta == 0)
  {
    stop();
    Serial.println("startup");
  }
  else if (x == 1000 && y == 2000 && theta == 3000 )
  {
    stop();
    Serial.println("stopping");
  }
  else if(x != 1000 && y != 2000 && theta != 3000)
  {
    // float error = desired - (int)heading;
    float error = 0;
    // float leftSpeed = 200 + kP*error;
    // float rightSpeed = 200 - kP*error;
    float leftSpeed = x;
    float rightSpeed = y;
    // Serial.print("left: ");
    // Serial.println(leftSpeed);
    // Serial.print("right: ");
    // Serial.println(rightSpeed);
    move_wheels(leftSpeed, rightSpeed);
    Serial.println("moving");
  }

  // else if (x == 69 && y == 69 && theta == 69)
  // {
  //   stop();
  //   Serial.println("port closed");
  // }
  else
  {
    stop();
    Serial.print("Error in bluetooth data: unknown value received: ");
    while(1);
  }

  delay(100);

  return;
}

