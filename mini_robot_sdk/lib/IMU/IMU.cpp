#include "IMU.h"

#include "bluetooth.h"

#include <Arduino.h>
#include <Wire.h> // I2C lib
#include <LSM6.h> // Accel and Gyro lib
#include <LIS3MDL.h> // Magnetometer lib
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

LSM6 gyroAcc; // creating accelerometer/gyro object
LIS3MDL mag;

// Constant variables
const float scaleA = 0.061;
const float scaleG = 4.375;
const float scaleM = 6842.0;
const float delta = 0.2;

// Sensor variables
float dt;
float ax, ay, az;
float ax0, ay0, az0;
float wx, wy, wz;
float mx, my, mz;
float magGlobalX, magGlobalY, magGlobalZ;
float magOffset[3], magGain[3];
Vector3d acc_th;
Vector3d gyro_th; 

// Angle Variables
int roll, pitch, yaw;
int roll0, pitch0, yaw0;
float phiDot, thetaDot, psiDot;
float phi, theta, psi;
float phi_0, theta_0, psi_0;
float psiOffset;

// Matrices and Position Variables
Vector3d a0;
Vector3d v0;
Vector3d x0;

int pop[30];
int avg = 0;

// delta variables
int counter = 0;
unsigned long oldTime = 0;

//________Gyro Offset Variables___________//
float gyro_pitch_cal = 0;
float gyro_roll_cal = 0;
float gyro_yaw_cal = 0;
float gyro_pitch = 0;
float gyro_roll = 0;
float gyro_yaw = 0;
int calib_cnt = 100;

//LP filter variables
const float alpha = 0.5; //bigger = not enough filtering, lower = too much filtering
double filtered_data[6] = {0, 0, 0, 0, 0, 0};
double data[3] = {0, 0, 0};

Vector3d v;
Vector3d a;

void initIMU() 
{
  Wire.begin();
  
  if (!gyroAcc.init() || !mag.init())
   {
     Serial.println("Failed to detect and initialize IMU");
     while(1); // preserves error message on serial monitor
   }
  gyroAcc.enableDefault();
  mag.enableDefault();
  mag.read();

  phi = 0;
  theta = 0;
  psi = 0;

  phi_0 = 0;
  theta_0 = 0;
  psi_0 = 0;

  psiOffset = 0;

  acc_th << 0,0,0;
  gyro_th << 0,0,0;

//_____________Calib Mag_________________//
  magOffset[0] = -15845; //mxMin
  magOffset[1] = 4232;   //myMin
  magOffset[2] = -4223;  //mzMin

  magGain[0] = (-6822 - -15845);  //mxMax - mxMin
  magGain[1] = (5568 - 4232);     //myMax - myMin
  magGain[2] = (13157 - -4223);   //mzMax - mzMin

//_____________Calib Gyro_________________//
  for(int i = 0; i < calib_cnt; i++)
  {
    gyroAcc.read();
    gyro_pitch_cal += gyroAcc.g.y;                              //Add pitch value to gyro_pitch_cal.
    gyro_roll_cal  += gyroAcc.g.x;                               //Add roll value to gyro_roll_cal.
    gyro_yaw_cal   += gyroAcc.g.z;                                //Add yaw value to gyro_yaw_cal.

    // Update acceleration thresholds
    if (abs(gyroAcc.a.x) > abs(acc_th(0)))
    {
      acc_th(0) = abs(gyroAcc.a.x);
    }
        if (abs(gyroAcc.a.y) > abs(acc_th(1)))
    {
      acc_th(1) = abs(gyroAcc.a.y);
    }
        if (abs(gyroAcc.a.z) > abs(acc_th(2)))
    {
      acc_th(2) = abs(gyroAcc.a.z);
    }

    // Update gyro thresholds
    if (abs(gyroAcc.g.x) > abs(gyro_th(0)))
    {
      gyro_th(0) = abs(gyroAcc.g.x);
    }
        if (abs(gyroAcc.g.y) > abs(gyro_th(1)))
    {
      gyro_th(1) = abs(gyroAcc.g.y);
    }
        if (abs(gyroAcc.g.z) > abs(gyro_th(2)))
    {
      gyro_th(2) = abs(gyroAcc.g.z);
    }

    delay(100);
  }

  gyro_pitch_cal /= calib_cnt;
  gyro_roll_cal  /= calib_cnt;
  gyro_yaw_cal   /= calib_cnt;

  // Serial.println("Calibration things:"); // global variables from here are accesible in main and other files :)
  // Serial.println(gyro_roll_cal);
  // Serial.println(gyro_pitch_cal);
  // Serial.println(gyro_yaw_cal);
}

// void NMNI()
// {
//   // if imu is static (if IMU readings are below a threshold)
//   // -- sample for max acceleration readings
//   // -- set max readings to be new threshold

//   // need to figure out how to resample to find a new threshold
//   // check how long the data is above our threshold? if it just jumps back
//   // forth then it is not moving, just giving us a new threshold?

//   // gyroAcc.read();

//   // set acc data to 0
//   if (abs(gyroAcc.a.x) < abs(acc_th(0)))
//   {
//     ax = 0;
//     v(0) = 0;
//     a(0) = 0;
//   }
//   if (abs(gyroAcc.a.y) < abs(acc_th(1)))
//   {
//     ay = 0;
//     v(1) = 0;
//     a(1) = 0;
//   }
//   if (abs(gyroAcc.a.z) < abs(acc_th(2)))
//   {
//     az = 0;
//     v(2) = 0;
//     a(2) = 0;
//   }

//   // // Update acceleration thresholds
//   // if (abs(gyroAcc.a.x) > abs(acc_th(0)))
//   // {
//   //   acc_th(0) = abs(gyroAcc.a.x);
//   // }
//   //     if (abs(gyroAcc.a.y) > abs(acc_th(1)))
//   // {
//   //   acc_th(1) = abs(gyroAcc.a.y);
//   // }
//   //     if (abs(gyroAcc.a.z) > abs(acc_th(2)))
//   // {
//   //   acc_th(2) = abs(gyroAcc.a.z);
//   // }

//   return;
// }

// void NDZTA()
// {

//   return;
// }

Vector3d get_acc(){
  Vector3d a;
  a << ax, ay, az;
  return a;
}

Vector3d get_rpy()
{
  gyroAcc.read();
  mag.read();

  dt = float(CalculateDeltaTime()) / 1000.0;

//________Gyro data and conversion_______//
  wx = gyroAcc.g.x;
  wy = gyroAcc.g.y;
  wz = gyroAcc.g.z;

  wy -= gyro_pitch_cal; // apply calibration
  wx -= gyro_roll_cal;
  wz -= gyro_yaw_cal; 

  wy *= scaleG / 1000.0 * M_PI/180.0; // convert to deg/s then rads
  wx *= scaleG / 1000.0 * M_PI/180.0;
  wz *= scaleG / 1000.0 * M_PI/180.0; 
  
//________________Acc data (cm/s^2)_______________________//
  ax = gyroAcc.a.x * scaleA / 100.0;
  ay = gyroAcc.a.y * scaleA / 100.0;
  az = gyroAcc.a.z * scaleA / 100.0;

  // NMNI(); // filtering acceleration readings

  data[0] = ax;
  data[1] = ay;
  data[2] = az; 

  filtered_data[3] = alpha * data[0] + (1 - alpha) * filtered_data[0];
  filtered_data[4] = alpha * data[1] + (1 - alpha) * filtered_data[1];
  filtered_data[5] = alpha * data[2] + (1 - alpha) * filtered_data[2];

  filtered_data[0] = filtered_data[3];
  filtered_data[1] = filtered_data[4];
  filtered_data[2] = filtered_data[5];

  ax = filtered_data[3];
  ay = filtered_data[4];
  az = filtered_data[5];
  
//________________Mag data (uT)__________________________//

  mx = ((mag.m.x  - magOffset[0]) / magGain[0]) / scaleM * -1;
  my = ((mag.m.y  - magOffset[1]) / magGain[1]) / scaleM * -1;
  mz = ((mag.m.z  - magOffset[2]) / magGain[2]) / scaleM * -1;

  // Gyro angular velocity
  phiDot = (wx + (wz*cos(phi) + wy*sin(phi))*tan(theta));
  thetaDot = (wy*cos(phi) - wz*sin(phi));
  psiDot = ((wz*cos(phi) + wy*sin(phi))/cos(theta));

  // Gyro numerical integration
  phi_0 = phiDot*dt + phi_0;
  theta_0 = thetaDot*dt + theta_0;
  psi_0 = psiDot*delta + psi_0;
  
  // Accel and Mag angles
  phi = (atan2(-ay, -az));                                              // converting to degs
  theta = (atan2(ax, sqrt(ay*ay + az*az)));
  psi = ((atan2(mx*cos(theta) + my*sin(theta)*sin(phi) + mz*sin(theta)*cos(phi), my*cos(phi) - mz*sin(phi))));

  // Complementary Filtering 
  roll = (0.02*phi_0 + 0.98*phi) * 180.0/M_PI;
  pitch = (0.02*theta_0 + 0.98*theta) * 180.0/M_PI;
  yaw = (0.98*psi_0 + 0.02*psi) * 180.0/M_PI;

  if(counter < 1) // Setting ICs
  {
    roll0 = roll;
    pitch0 = pitch;
    yaw0  = yaw;
    counter = 1000;
    ax0 = ax * -1;
    ay0 = ay * -1;
    az0 = az * -1; // redefine these for position
    psi_0 = psi;
  }

  roll = roll - roll0; //x
  pitch = pitch - pitch0; //y
  yaw = yaw - yaw0; //z

  Vector3d angles;
  angles << roll, pitch, yaw;
  // angles << dt, dt, dt;

  return angles;
}

Vector3d get_position(Vector3d angles)
{
  //_____________POSITION_CALCULATIONS___________________//
  float cPhi = cos(angles(0) * M_PI/180);
  float sPhi = sin(angles(0) * M_PI/180);
  float cTh = cos(angles(1) * M_PI/180);
  float sTh = sin(angles(1) * M_PI/180);
  float cPsi = cos(angles(2) * M_PI/180);
  float sPsi = sin(angles(2) * M_PI/180);
  
  Matrix3d R;
  R << cTh*cPsi, sPhi*sTh*cPsi - cPhi*sPsi, cPhi*sTh*cPsi + sPhi*sPsi,
       cTh*sPsi, sPhi*sTh*sPsi + cPhi*cPsi, cPhi*sTh*sPsi - sPhi*cPsi,
           -sTh,                  sPhi*cTh,                  cPhi*cTh;
  Matrix3d R_T = R.transpose();


  Matrix3d W;
  W << 0 , -1* wz * 180/M_PI, wy * 180/M_PI,
       wz * 180/M_PI, 0, -1 * wx * 180/M_PI,
       -1 * wy * 180/M_PI, wx * 180/M_PI, 0;
  
  Vector3d acc;
  Vector3d temp;
  // calculating linear acceleration
  Vector3d acc_linear;
  temp << ax, ay, az;
  acc_linear = R * acc;

  // calculating gravitational acceleration
  Vector3d acc_grav;
  acc << 0, 0, az0;
  acc_grav = R_T * acc;

  // calculating centripetal acceleration
  Vector3d acc_centrp;
  acc << v0(0), v0(1), v0(2);
  acc_centrp = W * acc;

  a = acc_linear; // + acc_grav + acc_centrp;
  
  // Integration from acc to vel
  v = a*dt + v0;

  // Integration from vel to pos
  Vector3d s = 0.5 * a * dt*dt + v*dt + x0;

  byte* pose = getBT();

  s(0) = pose[0] + pose[1]*256;
  s(1) = pose[2] + pose[3]*256;

  // setting ICs
  a0 = a;
  v0 = v;
  x0 = s;

  return s; 
}

VectorXd get_pose() 
{
  Vector3d rpy;
  rpy = get_rpy();

  Vector3d position;
  position = get_position(rpy);

  VectorXd pose{6};
  pose << position, rpy;

  return pose;
}

//__________________Math Functions_____________________________//

Vector3d trapezoidal(Vector3d s1, Vector3d s0, Vector3d ic)
{
  Vector3d s;
  float dt = float(CalculateDeltaTime()) / 1000.0;  

    s << (s1(0) + s0(0)) * dt * 0.5 + ic(0),
         (s1(1) + s0(1)) * dt * 0.5 + ic(1),
         (s1(2) + s0(2)) * dt * 0.5 + ic(2);
  
  return s;
}

unsigned long CalculateDeltaTime()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = (currentTime - oldTime);
  oldTime = currentTime;
  return deltaTime;
}
