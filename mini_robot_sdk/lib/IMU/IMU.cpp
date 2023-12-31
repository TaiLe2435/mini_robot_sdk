#include "IMU.h"

#include <Arduino.h>
#include <Wire.h> // I2C lib
#include <LSM6.h> // Accel and Gyro lib
#include <LIS3MDL.h> // Magnetometer lib

LSM6 gyroAcc; // creating accelerometer/gyro object
LIS3MDL mag;

// Constant variables
const float scaleA {0.061};
const float scaleG {4.375};
const float scaleM {6842.0};
const float delta {0.2};

// Sensor variables
float dt;
float ax, ay, az;
float ax0, ay0, az0;
float wx, wy, wz;
float mx, my, mz;
float magGlobalX, magGlobalY, magGlobalZ;
float magOffset[3], magGain[3];

// Magnetometer calibration constants
const float magOffsetX {-3764.0}; //mxMin
const float magOffsetY {2220.0};   //myMin
const float magOffsetZ {-4925.0};  //mzMin

const float magGainX = 4466.0 - -3764.0; //mxMax - mxMin
const float magGainY = 10532 - 2220.0;    //myMax - myMin
const float magGainZ = 3402.0 - -4925.0;  //mzMax - mzMin


// Angle Variables
int a, b, drift;
int roll, pitch, yaw;
int roll0, pitch0, yaw0;
float phiDot, thetaDot, psiDot;
float phi, theta, psi;
float phi_0, theta_0, psi_0, psi_IC;
float psiOffset;

// Matrices and Position Variables
float cPhi, sPhi, cTh, sTh, cPsi, sPsi;
float R[3][3] {{ 0 }};
float T[3][3] {{ 0 }};
float W[3][3] {{ 0 }};
int A[3][1] {{ 0 }};
float aT[3][1] {{ 0 }};
float aG[3][1] {{ 0 }};
float aCentrp[3][1] {{ 0 }};
int a0[3][1] {{ 0 }};
int v0[3][1] {{ 0 }};
int x0[3][1] {{ 0 }};
int s[3][1] {{ 0 }};
int zero[3][1] {{ 0 }};

int pop[30];
int avg {0};

// delta variables
int counter {0};
unsigned long oldTime {0};

//________Gyro Offset Variables___________//
float gyro_pitch_cal {0};
float gyro_roll_cal {0};
float gyro_yaw_cal {0};
float gyro_pitch {0};
float gyro_roll {0};
float gyro_yaw {0};
int calib_cnt {1000};
float calib[3];

//LP filter variables
const float alpha {0.5}; //bigger = not enough filtering, lower = too much filtering
double filtered_data[6] {0};
double data[3] {0};

float* initIMU();
float poseEstimation(float gyro_roll_cal, float gyro_pitch_cal, float gyro_yaw_cal);
void transform(float accX, float accY, float accZ, float matrix[3][3], String acc);
void transpose(float matrix[3][3]);
void euler(int sDdot[3][1], int sdot[3][1], int s0[3][1]);
void simpsons();
void trapezoidal(int s1[3][1], int s0[3][1], int ic[3][1]);
void popAvg(int newData);
unsigned long CalculateDeltaTime();

float* initIMU() 
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


//_____________Calib Gyro_________________//
  for(int i = 0; i < calib_cnt; i++)
  {
    gyroAcc.read();
    gyro_pitch_cal += gyroAcc.g.y;                              //Add pitch value to gyro_pitch_cal.
    gyro_roll_cal  += gyroAcc.g.x;                               //Add roll value to gyro_roll_cal.
    gyro_yaw_cal   += gyroAcc.g.z;                                //Add yaw value to gyro_yaw_cal.
  }

  gyro_pitch_cal /= calib_cnt;
  gyro_roll_cal  /= calib_cnt;
  gyro_yaw_cal   /= calib_cnt;

  calib[0] = gyro_roll_cal;
  calib[1] = gyro_pitch_cal;
  calib[2] = gyro_yaw_cal;

  return calib;  
}

float poseEstimation(float gyro_roll_cal, float gyro_pitch_cal, float gyro_yaw_cal) 
{
  
  mag.read();

  dt = float(CalculateDeltaTime()) / 10000.0;

//________Gyro data and conversion_______//
  wx = gyroAcc.g.x;
  wy = gyroAcc.g.y;
  wz = gyroAcc.g.z;

  wy -= gyro_pitch_cal; // apply calibration
  wx -= gyro_roll_cal;
  wz -= gyro_yaw_cal; 

  wy *= scaleG / 1000.0 * M_PI/180.0 * -1; // convert to deg/s then rads
  wx *= scaleG / 1000.0 * M_PI/180.0;
  wz *= scaleG / 1000.0 * M_PI/180.0 * -1; 
  
//________________Acc data (cm/s^2)_______________________//
  ax = gyroAcc.a.x * scaleA / 100.0 * -1;
  ay = gyroAcc.a.y * scaleA / 100.0;
  az = gyroAcc.a.z * scaleA / 100.0 * -1;

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

  mx = ((mag.m.x  - magOffsetX) / magGainX) / scaleM;
  my = ((mag.m.y  - magOffsetY) / magGainY) / scaleM;
  mz = ((mag.m.z  - magOffsetZ) / magGainZ) / scaleM;

  if(counter < 1) // Setting ICs
  {
    roll0 = roll;
    pitch0 = pitch;
    yaw0  = yaw;
    counter = 1000;
    ax0 = ax * -1;
    ay0 = ay * -1;
    az0 = az * -1; // redefine these for position
    psi_IC = ((atan2(mx*cos(theta) + my*sin(theta)*sin(phi) + mz*sin(theta)*cos(phi), my*cos(phi) - mz*sin(phi))));
    psi_0 = 0;
  }

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
  psi = ((atan2(mx*cos(theta) + my*sin(theta)*sin(phi) + mz*sin(theta)*cos(phi), my*cos(phi) - mz*sin(phi)))) - psi_IC;

  // Complementary Filtering 
  roll = (0.02*phi_0 + 0.98*phi) * 180.0/M_PI;
  pitch = (0.02*theta_0 + 0.98*theta) * 180.0/M_PI;
  yaw = (0.80*psi_0 + 0.20*psi) * 180.0/M_PI;

  roll = roll - roll0; //x
  pitch = pitch - pitch0; //y
  yaw = yaw - yaw0; //z

//_____________POSITION_CALCULATIONS___________________//

  // cPhi = cos(roll * M_PI/180);
  // sPhi = sin(roll * M_PI/180);
  // cTh = cos(pitch * M_PI/180);
  // sTh = sin(pitch * M_PI/180);
  // cPsi = cos(yaw * M_PI/180);
  // sPsi = sin(yaw * M_PI/180);
  
  // R[0][0] = cTh*cPsi;
  // R[0][1] = sPhi*sTh*cPsi - cPhi*sPsi;
  // R[0][2] = cPhi*sTh*cPsi + sPhi*sPsi;
  // R[1][0] = cTh*sPsi;
  // R[1][1] = sPhi*sTh*sPsi + cPhi*cPsi;
  // R[1][2] = cPhi*sTh*sPsi - sPhi*cPsi;
  // R[2][0] = -sTh;
  // R[2][1] = sPhi*cTh;
  // R[2][2] = cPhi*cTh;

  // //Matrix operations
  // W[0][0] = 0;
  // W[0][1] = -1* wz * 180/M_PI;
  // W[0][2] = wy * 180/M_PI;
  // W[1][0] = wz * 180/M_PI;
  // W[1][1] = 0;
  // W[1][2] = -1 * wx * 180/M_PI;
  // W[2][0] = -1 * wy * 180/M_PI;
  // W[2][1] = wx * 180/M_PI;
  // W[2][2] = 0;
  
  // transpose(R);
  // transform(ax+ax0, ay+ay0, az+az0, R, "linear");
  // transform(0, 0, az0, T, "gravity");
  // transform(v0[0][0], v0[1][0], v0[2][0], W, "centripetal");

  // if(abs(aT[0][0]) < 0.2)
  // {
  //   aT[0][0] = 0.0;
  // }
  // if(abs(aT[1][0]) < 0.2)
  // {
  //   aT[1][0] = 0.0;
  // }
  // if(abs(aT[2][0]) < 0.6)
  // {
  //   aT[2][0] = 0.0;
  // }

  // A[0][0] = aT[0][0] * 100; // + aG[0][0]*1000; // + aCentrp[0][0]/10;
  // A[1][0] = aT[1][0] * 100; // + aG[1][0]*1000; // + aCentrp[1][0]/10;
  // A[2][0] = aT[2][0] * 100; // + aG[2][0]*1000; // + aCentrp[2][0]/10;

//  Serial.print("Linear Acc: ");
//  Serial.print(aT[0][0]);
//  Serial.print(" ");
//  Serial.print(aT[1][0]);
//  Serial.print(" ");
//  Serial.println(aT[2][0]);
  
  //Integration from acc to vel
//   trapezoidal(A, a0, v0);
//   a0[0][0] = A[0][0];
//   a0[1][0] = A[1][0];
//   a0[2][0] = A[2][0];
// //  euler(zero, A, v0);
//   if(v0[0][0] == s[0][0])
//   {
//     v0[0][0] = 0;
//     s[0][0] = 0;
//   }
//     if(v0[1][0] == s[1][0])
//   {
//     v0[1][0] = 0;
//     s[1][0] = 0;
//   }
//     if(v0[2][0] == s[2][0])
//   {
//     v0[2][0] = 0;
//     s[2][0] = 0;
//   }
  
//   v0[0][0] = s[0][0];
//   v0[1][0] = s[1][0];
//   v0[2][0] = s[2][0];

//  Serial.print("Velocity: ");
//  Serial.print(s[0][0]);
//  Serial.print(" ");
//  Serial.print(s[1][0]);
//  Serial.print(" ");
//  Serial.println(s[2][0]);
  
  //Integration from vel to pos
  // trapezoidal(s, v0, x0);
//  euler(A, s, x0);
  // x0[0][0] = s[0][0];
  // x0[1][0] = s[1][0];
  // x0[2][0] = s[2][0];

  // popAvg(s[1][0]);

//_________________________Print Statements___________________________//

//  Serial.print("Position: ");
// //  Serial.print(s[0][0]);
// //  Serial.print(" ");
//  Serial.println(avg);
// //  Serial.print(" ");
// //  Serial.println(s[2][0]);

  // Serial.print("Angles: ");
  // Serial.print(roll);
  // Serial.print(" ");
  // Serial.print(pitch);
  // Serial.print(" ");
  // Serial.println(yaw);

  // delay(100);
  return yaw; // check delta, check if changing psi_0 to int changes drift
}

//__________________Functions_____________________________//
void transform(float accX, float accY, float accZ, float matrix[3][3], String acc )
{
  if(acc == "linear"){
  aT[0][0] = matrix[0][0]*accX + matrix[0][1]*accY + matrix[0][2]*accZ;
  aT[1][0] = matrix[1][0]*accX + matrix[1][1]*accY + matrix[1][2]*accZ;
  aT[2][0] = matrix[2][0]*accX + matrix[2][1]*accY + matrix[2][2]*accZ;
  }

  if(acc == "gravity"){
  aG[0][0] = matrix[0][0]*accX + matrix[0][1]*accY + matrix[0][2]*accZ;
  aG[1][0] = matrix[1][0]*accX + matrix[1][1]*accY + matrix[1][2]*accZ;
  aG[2][0] = matrix[2][0]*accX + matrix[2][1]*accY + matrix[2][2]*accZ;
  }

  if(acc == "centripetal"){
  aCentrp[0][0] = matrix[0][0]*accX + matrix[0][1]*accY + matrix[0][2]*accZ;
  aCentrp[1][0] = matrix[1][0]*accX + matrix[1][1]*accY + matrix[1][2]*accZ;
  aCentrp[2][0] = matrix[2][0]*accX + matrix[2][1]*accY + matrix[2][2]*accZ;
  }
  
  return;
}

void transpose(float matrix[3][3])
{

  T[0][0] = matrix[0][0];
  T[0][1] = matrix[1][0];
  T[0][2] = matrix[2][0];
  T[1][0] = matrix[0][1];
  T[1][1] = matrix[1][1];
  T[1][2] = matrix[2][1];
  T[2][0] = matrix[0][2];
  T[2][1] = matrix[1][2];
  T[2][2] = matrix[2][2];
  
  return;
}

void euler(int sDdot[3][1], int sdot[3][1], int s0[3][1])
{
  
  s[0][0] = 0.5 * sDdot[0][0] * dt*dt + sdot[0][0]*dt + s0[0][0];
  s[1][0] = 0.5 * sDdot[1][0] * dt*dt + sdot[1][0]*dt + s0[1][0];
  s[2][0] = 0.5 * sDdot[2][0] * dt*dt + sdot[2][0]*dt + s0[2][0];

//  s[0][0] = sdot[0][0]*dt + s0[0][0];
//  s[1][0] = sdot[1][0]*dt + s0[1][0];
//  s[2][0] = sdot[2][0]*dt + s0[2][0];
  
  return;
}

void simpsons()
{
  // not full implemented yet
  return;
}

void trapezoidal(int s1[3][1], int s0[3][1], int ic[3][1])
{

  s[0][0] = (s1[0][0] + s0[0][0]) * dt * 0.5 + ic[0][0];
  s[1][0] = (s1[1][0] + s0[1][0]) * dt * 0.5 + ic[1][0];
  s[2][0] = (s1[2][0] + s0[2][0]) * dt * 0.5 + ic[2][0];
  
  return;
}

void popAvg(int newData)
{

  for(int j = 3; j > 1; j--)
  {
    pop[j] = pop[j-1];
  }

  pop[0] = newData;
  
  for(int i = 0; i < 3; i++)
  {
    avg += pop[i];
  }

  avg = avg/3;
  
  return;
}

unsigned long CalculateDeltaTime()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = (currentTime - oldTime);
  oldTime = currentTime;
  return deltaTime;
}
