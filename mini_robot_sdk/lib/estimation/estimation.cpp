#include "estimation.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h> // I2C lib
#include <LSM6.h> // Accel and Gyro lib
#include <LIS3MDL.h> // Magnetometer lib
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

LSM6 gyroAcc; 
LIS3MDL mag;

unsigned long oldTime {0}; // used to calculate time step

Estimation::Estimation(){
    
}

//_____________________________IMU DRIVERS__________________________________________//

float* Estimation::calib_gyro(){
    Wire.begin();

    if (!gyroAcc.init())
    {
        Serial.println("Failed to detect and initialize IMU");
        while(1); // preserves error message on serial monitor
    }
    gyroAcc.enableDefault();

    phi = 0;
    theta = 0;
    psi = 0;

    phi_0 = 0;
    theta_0 = 0;
    psi_0 = 0;

    psiOffset = 0;

    int calib_cnt {1000};
    //_____________Calib Gyro_________________//
    for(int i = 0; i < calib_cnt; i++)
    {
        gyroAcc.read();
        calib[1] += gyroAcc.g.y;                              //Add pitch value to gyro_pitch_cal.
        calib[0]  += gyroAcc.g.x;                               //Add roll value to gyro_roll_cal.
        calib[2]  += gyroAcc.g.z;                                //Add yaw value to gyro_yaw_cal.
    }

    calib[1] /= calib_cnt;
    calib[0]  /= calib_cnt;
    calib[2]   /= calib_cnt;

    return calib;  
}

float* Estimation::get_gyro(float* gyro_calib){
    gyroAcc.read();

    const float scaleG {4.375};

    //________Gyro data and conversion_______//
    gyro[0] = gyroAcc.g.x;
    gyro[1] = gyroAcc.g.y;
    gyro[2] = gyroAcc.g.z;

    gyro[1] -= gyro_calib[1]; // apply calibration
    gyro[0] -= gyro_calib[0];
    gyro[2] -= gyro_calib[2]; 

    gyro[1] *= scaleG / 1000.0 * M_PI/180.0 * -1; // convert to deg/s then rads
    gyro[0] *= scaleG / 1000.0 * M_PI/180.0;
    gyro[2] *= scaleG / 1000.0 * M_PI/180.0 * -1; 

    return gyro;
}

float* Estimation::get_acc(){
    gyroAcc.read();

    //LP filter variables
    const float alpha {0.5}; //bigger = not enough filtering, lower = too much filtering
    const float scaleA {0.061};
    double filtered_data[6] {0};
    double data[3] {0};

    //________________Acc data (cm/s^2)_______________________//
    acc[0] = gyroAcc.a.x * scaleA / 100.0 * -1;
    acc[1] = gyroAcc.a.y * scaleA / 100.0;
    acc[2] = gyroAcc.a.z * scaleA / 100.0 * -1;

    data[0] = acc[0];
    data[1] = acc[1];
    data[2] = acc[2]; 

    filtered_data[3] = alpha * data[0] + (1 - alpha) * filtered_data[0];
    filtered_data[4] = alpha * data[1] + (1 - alpha) * filtered_data[1];
    filtered_data[5] = alpha * data[2] + (1 - alpha) * filtered_data[2];

    filtered_data[0] = filtered_data[3];
    filtered_data[1] = filtered_data[4];
    filtered_data[2] = filtered_data[5];

    acc[0] = filtered_data[3];
    acc[1] = filtered_data[4];
    acc[2] = filtered_data[5];

    return acc;
}

// Do I need this? Yes need rpy function that subtracts biases and estimates pose
// float* Estimation::get_rpy(){
// //     return 0.0;
// }

//_____________________________IMU ESTIMATION STUFF__________________________________________//

MatrixXd Estimation::imu_process_model(float* rpy, float* acc){
    const float betaG {0.2};
    const float betaA {0.2};

    float cR = cos(rpy[0]);
    float sR = sin(rpy[0]);
    float cP = cos(rpy[1]);
    float sP = sin(rpy[1]);
    float cY = cos(rpy[2]);
    float sY = sin(rpy[2]);

    Matrix3d C;
    C << cP*cY, sP*sR*cY-cR*sY, cR*sP*cY+sR*sY,
        cP*sY, sR*sP*sY+cR*cY, cR*sP*sY-cY*sR,
        -sP, sR*cP, cR*cP;

    Matrix3d S;
    S << 0, -acc[2], acc[1],
         acc[2], 0, -acc[0],
         -acc[1], acc[0], 0;
    
    // Fk(:,:,i) = [zero33 zero33    -C      zero33;
    //         S  zero33   zero33      C;
    //       zero33 zero33 -betaG*I33   zero33;
    //       zero33 zero33   zero33   -betaA*I33];

    Fk_imu.setZero();

    Fk_imu.block<3,3>(0,6) = -C;
    Fk_imu.block<3,3>(3,0) = S;
    Fk_imu.block<3,3>(3,9) = C;
    Fk_imu.block<3,3>(6,6) = -betaG * Matrix3d::Identity();
    Fk_imu.block<3,3>(9,9) = -betaA * Matrix3d::Identity();

    return Fk_imu;
}

MatrixXd Estimation::imu_process_noise(float* rpy){

    float cR = cos(rpy[0]);
    float sR = sin(rpy[0]);
    float cP = cos(rpy[1]);
    float sP = sin(rpy[1]);
    float cY = cos(rpy[2]);
    float sY = sin(rpy[2]);

    Matrix3d C;

    C << cP*cY, sP*sR*cY-cR*sY, cR*sP*cY+sR*sY,
         cP*sY, sR*sP*sY+cR*cY, cR*sP*sY-cY*sR,
         -sP, sR*cP, cR*cP; 
    
    // Gk(:,:,i) = [-C zero33 zero33 zero33;
    //             zero33 C zero33 zero33;
    //             zero33 zero33 I33 zero33;
    //             zero33 zero33 zero33 I33];

    Gk_imu.setZero();
    Gk_imu.block<3,3>(0,0) = -C;
    Gk_imu.block<3,3>(3,3) = C;
    Gk_imu.block<3,3>(6,6) = Matrix3d::Identity();
    Gk_imu.block<3,3>(9,9) = Matrix3d::Identity();

    return Gk_imu;
}

// not done
Vector4d Estimation::imu_measurement(float ddr_heading, float* pos_prev, float* a_0){
    // finding heading error
    // float* pose = get_rpy();
    float* pose = get_acc(); // waiting for get_rpy function or pose from other EKF
    
    float posx_current = pose[0];
    float posy_current = pose[1];
    
    float acc_heading = atan2(posx_current - pos_prev[0], posy_current-pos_prev[1]);
    float heading_error = ddr_heading - acc_heading;

    // finding velocities
    float* acc = get_acc();
    float dt = float(calculate_delta_time()) / 10000.0;

    float vel[3] {};
    vel[0] = acc[0] * dt + a_0[0];
    vel[1] = acc[1] * dt + a_0[1];
    vel[2] = acc[2] * dt + a_0[2];

    zk_imu<< heading_error, vel[0], vel[1], vel[2];

    return zk_imu;
}

MatrixXd Estimation::imu_measurement_model(float* rpy){

    float tR = tan(rpy[0]);
    float cY = cos(rpy[2]);
    float sY = sin(rpy[2]);

    Vector3d temp {tR*cY, tR*sY, -1};
    
    // Hjacobian(:,:,i) = [tR*cY tR*sY -1 zero13 zero13 zero13;
    //             zero33  I33   zero33 zero33];

    Hk_imu.setZero();
    Hk_imu.block<1,3>(0,0) = temp;
    Hk_imu.block<3,3>(1,3) = Matrix3d::Identity();

    return Hk_imu; 
}

void Estimation::imu_predict(){

}

void Estimation::imu_update(){
    
}

void Estimation::imu_ekf(){

}

//_____________________________ROBOT ESTIMATION STUFF__________________________________________//

Vector3d Estimation::unicycle_model(float v, float w){
    float dt = float(calculate_delta_time()) / 10000.0;
    
    f_ddr[0] += v*cos(f_ddr[2])*dt;
    f_ddr[1] += v*sin(f_ddr[2])*dt;
    f_ddr[2] += w*dt;

    return f_ddr;
}

// figure out which v to pass | theoretical v we command
Matrix3d Estimation::ddr_process(float v, float heading){
    float dt = float(calculate_delta_time()) / 10000.0;
    
    Fk_ddr << 1, 0, -v*sin(heading)*dt,
               0, 1, v*cos(heading)*dt,
               0, 0, 1;

    return Fk_ddr;
}

Vector3d Estimation::ddr_measurement(float* pose){ // could just call a pose() function inside so no parameters
    zk_ddr[0] = pose[0]; // x
    zk_ddr[1] = pose[1]; // y
    zk_ddr[2] = pose[5]; // theta

    return zk_ddr;
}

Matrix3d Estimation::ddr_measurement_model(){ // could just delete and set Hk_ddr as a constant that doesnt get accessed 
    Hk_ddr.setIdentity();

    return Hk_ddr;
}

void Estimation::ddr_predict(Matrix3d Fk, Matrix3d P, float v, float w){
    
    // States
    Vector3d states = unicycle_model(v, w);
    xk_ddr_prev[0] = states[0];
    xk_ddr_prev[1] = states[1];
    xk_ddr_prev[2] = states[2];

    // Covariance
    // P_prev = Fk*P*Fk' + Qk;
    Matrix3d Fk_T;
    Fk_T = Fk.transpose(); // Fk'
    P_ddr_prev = Fk*P*Fk_T + Qk_ddr;
}

void Estimation::ddr_update(Vector3d xk_prev, Vector3d xk, Matrix3d P_prev){

    Matrix3d Hk = ddr_measurement_model();

    // Innovation Covariance
    // S = Hk*P_*Hk' + R;

    Matrix3d S, Hk_T;
    Hk_T = Hk.transpose();
    S = Hk*P_prev*Hk_T + R_ddr;

    float pose[6] {}; // need to change this
    Vector3d zh = xk; // and this to Hk*xk
    Vector3d z = ddr_measurement(pose);

    Vector3d innovation;
    innovation << z[0] - zh[0], z[1] - zh[1], z[2] - zh[2]; // innov = z - H*x

    Matrix3d K;
    K = P_prev * Hk_T * S.inverse();

    xk_ddr = xk_prev + K*innovation;
    P_ddr = (Matrix3d::Identity() - K*Hk) * P_prev;

}

Vector3d Estimation::ddr_ekf(float v, float w, float* pose){
    Vector3d f = unicycle_model(v, w);

    Matrix3d Fk = ddr_process(v, pose[5]); // v and heading

    float dt = float(calculate_delta_time()) / 10000.0;
    MatrixXd Wk {3, 2};
    MatrixXd Wk_T {2, 3};
    Wk << cos(pose[5]) * dt, 0,
          sin(pose[3]) * dt, 0,
          0, dt;
    Wk_T = Wk.transpose();
    
    Matrix2d Qk;
    Qk << 0.05, 0,
         0 , 0.05;

    Matrix3d Q;
    Q = Wk * Qk * Wk_T;

    Vector3d h = ddr_measurement(pose);

    Matrix3d Hk = ddr_measurement_model();

    Matrix3d Rk;
    Rk << 0.2, 0, 0,
          0, 0.2, 0,
          0, 0, (M_PI/16)*(M_PI/16); 

    // gives xk_ddr and P_ddr members
    ddr_update(xk_ddr_prev, xk_ddr, P_ddr_prev);

    // gives xk_ddr_prev and P_ddr_prev members
    ddr_predict(Fk_ddr, P_ddr, v, w);

    return xk_ddr;
}

//_____________________________EXTRA STUFF__________________________________________//

unsigned long Estimation::calculate_delta_time()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = (currentTime - oldTime);
  oldTime = currentTime;
  return deltaTime;
}