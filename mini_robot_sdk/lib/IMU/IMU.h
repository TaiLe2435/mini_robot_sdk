#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

void initIMU();

void NMNI();

void NDZTA();

Vector3d get_rpy();

Vector3d get_position(Vector3d);

VectorXd get_pose(); 

Vector3d trapezoidal(Vector3d, Vector3d, Vector3d);

void popAvg(int);

unsigned long CalculateDeltaTime();

