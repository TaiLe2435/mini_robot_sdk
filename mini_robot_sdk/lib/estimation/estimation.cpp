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

//_____________________________IMU ESTIMATION STUFF__________________________________________//

MatrixXd Estimation::imu_process_model(Vector3d rpy, Vector3d acc){
    const float betaG {0.2};
    const float betaA {0.2};

    float cR = cos(rpy(0));
    float sR = sin(rpy(0));
    float cP = cos(rpy(1));
    float sP = sin(rpy(1));
    float cY = cos(rpy(2));
    float sY = sin(rpy(2));

    Matrix3d C;
    C << cP*cY, sP*sR*cY-cR*sY, cR*sP*cY+sR*sY,
        cP*sY, sR*sP*sY+cR*cY, cR*sP*sY-cY*sR,
        -sP, sR*cP, cR*cP;

    Matrix3d S;
    S << 0, -acc(2), acc(1),
         acc(2), 0, -acc(0),
         -acc(1), acc(0), 0;
    
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

MatrixXd Estimation::imu_process_noise(Vector3d rpy){

    float cR = cos(rpy(0));
    float sR = sin(rpy(0));
    float cP = cos(rpy(1));
    float sP = sin(rpy(1));
    float cY = cos(rpy(2));
    float sY = sin(rpy(2));

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
Vector4d Estimation::imu_measurement(float ddr_heading, Vector3d pos_prev, Vector3d a_0, VectorXd bias, Vector3d rpy_error){
    // finding heading error
    Vector3d pose = get_rpy(bias, rpy_error);
    // waiting for pose from other EKF
    
    float posx_current = pose(0);
    float posy_current = pose(1);
    
    float acc_heading = atan2(posx_current - pos_prev(0), posy_current-pos_prev(1));
    float heading_error = ddr_heading - acc_heading;

    // finding velocities
    Vector3d acc = get_acc();
    float dt = float(calculate_delta_time()) / 10000.0;

    float vel[3] {};
    vel[0] = acc(0) * dt + a_0(0);
    vel[1] = acc(1) * dt + a_0(1);
    vel[2] = acc(2) * dt + a_0(2);

    zk_imu<< heading_error, vel[0], vel[1], vel[2];

    return zk_imu;
}

MatrixXd Estimation::imu_measurement_model(Vector3d rpy){

    float tR = tan(rpy(0));
    float cY = cos(rpy(2));
    float sY = sin(rpy(2));

    Vector3d temp {tR*cY, tR*sY, -1};
    
    // Hjacobian(:,:,i) = [tR*cY tR*sY -1 zero13 zero13 zero13;
    //             zero33  I33   zero33 zero33];

    Hk_imu.setZero();
    Hk_imu.block<1,3>(0,0) = temp;
    Hk_imu.block<3,3>(1,3) = Matrix3d::Identity();

    return Hk_imu; 
}

void Estimation::imu_predict(VectorXd xk, MatrixXd Fk, MatrixXd P, MatrixXd Gk, MatrixXd Qk){
    xk_imu_prev = xk + Fk*(xk - xk_imu_prev);

    MatrixXd Fk_T{12,12};
    MatrixXd Gk_T{12,12};
    Fk_T = Fk.transpose();
    Gk_T = Gk.transpose();

    P_imu_prev = Fk*P*Fk_T + Gk*Qk*Gk_T;
}

void Estimation::imu_update(VectorXd xk_prev, VectorXd xk, MatrixXd P_prev, Vector3d rpy, Vector3d pos_prev, Vector3d a_0, VectorXd bias, Vector3d rpy_error){
    MatrixXd Hk{4,12}, Hk_T{12,4};
    Hk = imu_measurement_model(rpy);

    // Innovation Covariance
    // S = Hk*P_*Hk' + R;

    Matrix4d S{4,4};
    Hk_T = Hk.transpose();
    S = Hk*P_prev*Hk_T + R_imu;

    Vector4d zh;
    zh = Hk*xk;
    Vector4d z;
    z = imu_measurement(rpy[2], pos_prev, a_0, bias, rpy_error);

    Vector4d innovation;
    innovation << z[0] - zh[0], z[1] - zh[1], z[2] - zh[2], z[3] - zh[3]; // innov = z - H*x

    MatrixXd K{12,4};
    K = P_prev * Hk_T * S.inverse();

    xk_imu = xk_prev + K*innovation;
    MatrixXd I{12,12};
    P_imu = (I.setIdentity() - K*Hk) * P_prev;
}

VectorXd Estimation::imu_ekf(Vector3d rpy, Vector3d pos_prev, Vector3d a, Vector3d a_0, VectorXd bias, Vector3d rpy_error){
    MatrixXd Fk{12,12}, Gk{12,12}, Q{12,12};
    Fk = imu_process_model(rpy, a); // maybe make it so they're called in 
    Gk = imu_process_noise(rpy);    // predict and update

    Q.setZero();
    MatrixXd I{9,9};

    Q.block<3,3>(0,0) = M_PI/16 * Matrix3d::Identity();
    Q.block<9,9>(3,3) = 0.1 * I.setIdentity();
 
    R_imu << (M_PI/16)*(M_PI/16), 0, 0, 0,
                0, (0.6)*(0.6), 0, 0,
                0, 0, (0.6)*(0.6), 0,
                0, 0, 0, (0.6)*(0.6);

    // gives xk_imu and P_imu members
    imu_update(xk_imu_prev, xk_imu, P_imu_prev, rpy, pos_prev, a_0, bias, rpy_error);

    // gives xk_imu_prev and P_imu_prev members
    imu_predict(xk_imu, Fk, P_imu, Gk, Q);

    return xk_imu;
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

Vector3d Estimation::ddr_measurement(VectorXd pose){ // could just call a pose() function inside so no parameters
    zk_ddr[0] = pose(0); // x
    zk_ddr[1] = pose(1); // y
    zk_ddr[2] = pose(5); // theta

    return zk_ddr;
}

Matrix3d Estimation::ddr_measurement_model(){ // could just delete and set Hk_ddr as a constant that doesnt get accessed 
    Hk_ddr.setIdentity();

    return Hk_ddr;
}
// error stemming from here
void Estimation::ddr_predict(Matrix3d P, float v, float w, VectorXd pose){
    
    // States
    Vector3d states = unicycle_model(v, w);
    xk_ddr_prev[0] = states(0);
    xk_ddr_prev[1] = states(1);
    xk_ddr_prev[2] = states(2);

    // Covariance
    // // P_prev = Fk*P*Fk' + Qk;
    Matrix3d Fk, Fk_T;
    Fk = ddr_process(v, pose(5));
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

    VectorXd pose{6}; // need to change this
    Vector3d zh = xk; // and this to Hk*xk | no bc Hk is I
    Vector3d z = ddr_measurement(pose);

    Vector3d innovation;
    innovation << z[0] - zh[0], z[1] - zh[1], z[2] - zh[2]; // innov = z - H*x

    Matrix3d K;
    K = P_prev * Hk_T * S.inverse();

    xk_ddr = xk_prev + K*innovation;
    P_ddr = (Matrix3d::Identity() - K*Hk) * P_prev;

}

Vector3d Estimation::ddr_ekf(float v, float w, VectorXd pose){

    float dt = float(calculate_delta_time()) / 10000.0;
    MatrixXd Wk {3, 2};
    MatrixXd Wk_T {2, 3};
    Wk << cos(pose(5)) * dt, 0,
          sin(pose(3)) * dt, 0,
          0, dt;
    Wk_T = Wk.transpose();
    
    Matrix2d Q;
    Q << 0.05, 0,
         0 , 0.05;

    Qk_ddr = Wk * Q * Wk_T;

    R_ddr << 0.2, 0, 0,
          0, 0.2, 0,
          0, 0, (M_PI/16)*(M_PI/16); 

    // gives xk_ddr and P_ddr members
    ddr_update(xk_ddr_prev, xk_ddr, P_ddr_prev);

    // gives xk_ddr_prev and P_ddr_prev members
    ddr_predict(P_ddr, v, w, pose);

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