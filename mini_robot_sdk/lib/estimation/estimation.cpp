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

Estimation::MatrixNx12Pointer Estimation::imu_process_model(float* rpy, float* acc){
    const float betaG {0.2};
    const float betaA {0.2};

    float betaGI33[3][3] {
        {-betaG, 0, 0},
        {0, -betaG, 0},
        {0, 0, -betaG} 
    };

    float betaAI33[3][3] {
        { -betaA, 0, 0 },
        { 0, -betaA, 0 },
        { 0, 0, -betaA }
    };

    float zero33[3][3] {0};

    float cR = cos(rpy[0]);
    float sR = sin(rpy[0]);
    float cP = cos(rpy[1]);
    float sP = sin(rpy[1]);
    float cY = cos(rpy[2]);
    float sY = sin(rpy[2]);

    float C[3][3] {
        { cP*cY, sP*sR*cY-cR*sY, cR*sP*cY+sR*sY },
        { cP*sY, sR*sP*sY+cR*cY, cR*sP*sY-cY*sR },
        { -sP, sR*cP, cR*cP }
    };

    float neg_C[3][3] {
        { -cP*cY, -sP*sR*cY+cR*sY, -cR*sP*cY-sR*sY },
        { -cP*sY, -sR*sP*sY-cR*cY, -cR*sP*sY+cY*sR },
        { sP, -sR*cP, -cR*cP }
    };

    float S[3][3] {
        { 0, -acc[2], acc[1] },
        { acc[2], 0, -acc[0] },
        { -acc[1], acc[0], 0 }
    };

    
    // Fk(:,:,i) = [zero33 zero33    -C      zero33;
    //         S  zero33   zero33      C;
    //       zero33 zero33 -betaG*I33   zero33;
    //       zero33 zero33   zero33   -betaA*I33];

    insert_matrix_Nx12_default(Fk_imu, zero33, 0, 0);
    insert_matrix_Nx12_default(Fk_imu, zero33, 0, 3);
    insert_matrix_Nx12_default(Fk_imu, neg_C, 0, 6);
    insert_matrix_Nx12_default(Fk_imu, zero33, 0, 9);

    insert_matrix_Nx12_default(Fk_imu, S, 3, 0);
    insert_matrix_Nx12_default(Fk_imu, zero33, 3, 3);
    insert_matrix_Nx12_default(Fk_imu, zero33, 3, 6);
    insert_matrix_Nx12_default(Fk_imu, C, 3, 9);

    insert_matrix_Nx12_default(Fk_imu, zero33, 6, 0);
    insert_matrix_Nx12_default(Fk_imu, zero33, 6, 3);
    insert_matrix_Nx12_default(Fk_imu, betaGI33, 6, 6);
    insert_matrix_Nx12_default(Fk_imu, zero33, 6, 9);

    insert_matrix_Nx12_default(Fk_imu, zero33, 9, 0);
    insert_matrix_Nx12_default(Fk_imu, zero33, 9, 3);
    insert_matrix_Nx12_default(Fk_imu, zero33, 9, 6);
    insert_matrix_Nx12_default(Fk_imu, betaAI33, 9, 9);

    return Fk_imu;
}

Estimation::MatrixNx12Pointer Estimation::imu_process_noise(float* rpy){

float I33[3][3] {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1} 
    };

    float zero33[3][3] {0};

    float cR = cos(rpy[0]);
    float sR = sin(rpy[0]);
    float cP = cos(rpy[1]);
    float sP = sin(rpy[1]);
    float cY = cos(rpy[2]);
    float sY = sin(rpy[2]);

    float C[3][3] {
        { cP*cY, sP*sR*cY-cR*sY, cR*sP*cY+sR*sY },
        { cP*sY, sR*sP*sY+cR*cY, cR*sP*sY-cY*sR },
        { -sP, sR*cP, cR*cP }
    };

    float neg_C[3][3] {
        { -cP*cY, -sP*sR*cY+cR*sY, -cR*sP*cY-sR*sY },
        { -cP*sY, -sR*sP*sY-cR*cY, -cR*sP*sY+cY*sR },
        { sP, -sR*cP, -cR*cP }
    };

    
    // Gk(:,:,i) = [-C zero33 zero33 zero33;
    //             zero33 C zero33 zero33;
    //             zero33 zero33 I33 zero33;
    //             zero33 zero33 zero33 I33];

    insert_matrix_Nx12_default(Gk_imu, neg_C, 0, 0);
    insert_matrix_Nx12_default(Gk_imu, zero33, 0, 3);
    insert_matrix_Nx12_default(Gk_imu, zero33, 0, 6);
    insert_matrix_Nx12_default(Gk_imu, zero33, 0, 9);

    insert_matrix_Nx12_default(Gk_imu, zero33, 3, 0);
    insert_matrix_Nx12_default(Gk_imu, C, 3, 3);
    insert_matrix_Nx12_default(Gk_imu, zero33, 3, 6);
    insert_matrix_Nx12_default(Gk_imu, zero33, 3, 9);

    insert_matrix_Nx12_default(Gk_imu, zero33, 6, 0);
    insert_matrix_Nx12_default(Gk_imu, zero33, 6, 3);
    insert_matrix_Nx12_default(Gk_imu, I33, 6, 6);
    insert_matrix_Nx12_default(Gk_imu, zero33, 6, 9);

    insert_matrix_Nx12_default(Gk_imu, zero33, 9, 0);
    insert_matrix_Nx12_default(Gk_imu, zero33, 9, 3);
    insert_matrix_Nx12_default(Gk_imu, zero33, 9, 6);
    insert_matrix_Nx12_default(Gk_imu, I33, 9, 9);

    return Gk_imu;
}

// not done
float* Estimation::imu_measurement(float ddr_heading, float* pos_prev, float* a_0){
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

    zk_imu[0] = heading_error;
    zk_imu[1] = vel[0];
    zk_imu[2] = vel[1];
    zk_imu[3] = vel[2];

    return zk_imu;
}

Estimation::MatrixNx12Pointer Estimation::imu_measurement_model(float* rpy){
   float I13[1][3] {
        {1, 0, 0}
    };
   
   float I33[3][3] {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1} 
    };

    float zero33[3][3] {0};
    float zero13[1][3] {0};

    float tR = tan(rpy[0]);
    float cY = cos(rpy[2]);
    float sY = sin(rpy[2]);

    float temp[1][3] {tR*cY, tR*sY, -1};
    
    // Hjacobian(:,:,i) = [tR*cY tR*sY -1 zero13 zero13 zero13;
    //             zero33  I33   zero33 zero33];

    insert_matrix_Nx12(Hk_imu, temp, 4, 12, 1, 3, 0, 0);
    insert_matrix_Nx12(Hk_imu, zero13, 4, 12, 1, 3, 0, 3);
    insert_matrix_Nx12(Hk_imu, zero13, 4, 12, 1, 3, 0, 6);
    insert_matrix_Nx12(Hk_imu, zero13, 4, 12, 1, 3, 0, 9);

    insert_matrix_Nx12(Hk_imu, zero33, 4, 12, 3, 3, 1, 0);
    insert_matrix_Nx12(Hk_imu, I33, 4, 12, 3, 3, 1, 3);
    insert_matrix_Nx12(Hk_imu, zero33, 4, 12, 3, 3, 1, 6);
    insert_matrix_Nx12(Hk_imu, zero33, 4, 12, 3, 3, 1, 9);

    return Hk_imu; 
}

void Estimation::imu_predict(){

}

void Estimation::imu_update(){
    
}

void Estimation::imu_ekf(){

}

//_____________________________ROBOT ESTIMATION STUFF__________________________________________//

float* Estimation::unicycle_model(float v, float w){
    float dt = float(calculate_delta_time()) / 10000.0;
    
    f_ddr[0] += v*cos(f_ddr[2])*dt;
    f_ddr[1] += v*sin(f_ddr[2])*dt;
    f_ddr[2] += w*dt;

    return f_ddr;
}

// figure out which v to pass | theoretical v we command
Estimation::MatrixNx3Pointer Estimation::ddr_process(float v, float heading){
    float dt = float(calculate_delta_time()) / 10000.0;
    
    float ddr_stm [3][3] = {
        {1, 0, -v*sin(heading)*dt},
        {0, 1, v*cos(heading)*dt},
        {0, 0, 1} 
    };

    insert_matrix_Nx3(Fk_ddr, ddr_stm, 0, 0);

    return Fk_ddr;
}

float* Estimation::ddr_measurement(float* pose){ // could just call a pose() function inside so no parameters
    zk_ddr[0] = pose[0]; // x
    zk_ddr[1] = pose[1]; // y
    zk_ddr[2] = pose[5]; // theta

    return zk_ddr;
}

Estimation::MatrixNx3Pointer Estimation::ddr_measurement_model(){
    float I33[3][3] {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1} 
    };

    insert_matrix_Nx3(Hk_ddr, I33, 0, 0);

    return Hk_ddr;
}

void Estimation::ddr_predict(float Fk[][3], float P[][3], float v, float w){
    
    // States
    float* states = unicycle_model(v, w);
    xk_ddr_prev[0] = states[0];
    xk_ddr_prev[1] = states[1];
    xk_ddr_prev[2] = states[2];

    // Covariance
    // P_prev = Fk*P*Fk' + Qk;
    float Fk_T[3][3] {};
    matrix_transpose(Fk, Fk_T); // Fk'
    matrix_multiply(Fk, P, P_ddr_prev); // Fk*P
    matrix_multiply(P_ddr_prev, Fk_T, P_ddr_prev); // Fk*P*Fk'
    matrix_add(P_ddr_prev, Qk_ddr, P_ddr_prev); // Fk*P*Fk' + Qk
}

void Estimation::ddr_update(float* xk_prev, float* xk, float P_prev[][3]){

    MatrixNx3Pointer Hk = ddr_measurement_model();

    // Innovation Covariance
    // S = Hk*P_*Hk' + R;

    float S[3][3], Hk_T[3][3];
    matrix_transpose(Hk, Hk_T);
    matrix_multiply(Hk, P_prev, S);
    matrix_multiply(S, Hk_T, S);
    matrix_add(S, R_ddr, S);

    float pose[6] {}; // need to change this
    float* zh = xk; // and this to Hk*xk
    float* z = ddr_measurement(pose);

    float innovation[3] {};
    innovation[0] = z[0] - zh[0]; // innov = z - H*x
    innovation[1] = z[1] - zh[1];
    innovation[0] = z[2] - zh[2];

    float K[3][3] {};
    matrix_multiply(P_prev, Hk_T, K);


}

void Estimation::ddr_ekf(){

}

//_____________________________MATRIX STUFF__________________________________________//

void Estimation::insert_matrix_Nx12(float largeMatrix[][12], float smallMatrix[][3], int numRowsLg, int numColsLg, int numRowsSm, int numColsSm, int row, int col) {
    for (int i = 0; i < numRowsSm; ++i) {
        for (int j = 0; j < numColsSm; ++j) {
            largeMatrix[row + i][col + j] = smallMatrix[i][j];
        }
    }
}

void Estimation::insert_matrix_Nx12_default(float largeMatrix[][12], float smallMatrix[][3], int row, int col) {
    insert_matrix_Nx12(largeMatrix, smallMatrix, 12, 12, 3, 3, row, col);
}

void Estimation::insert_matrix_Nx3(float largeMatrix[][3], float smallMatrix[][3], int row, int col) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            largeMatrix[row + i][col + j] = smallMatrix[i][j];
        }
    }
}

// Function to multiply two matrices (result = A * B)
void Estimation::matrix_multiply(float A[][3], float B[][3], float result[][3]) {
    int i, j, k;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            result[i][j] = 0.0;
            for (k = 0; k < 3; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Function to add two matrices (result = A + B)
void Estimation::matrix_add(float A[][3], float B[][3], float result[][3]) {
    int i, j;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

// Function to transpose a matrix (result = A')
void Estimation::matrix_transpose(float A[][3], float result[][3]) {
    int i, j;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            result[i][j] = A[j][i];
        }
    }
}

Matrix3d Estimation::matrix_test(){
    // float I33[3][3] {
    //     {1, 2, 3},
    //     {4, 5, 6},
    //     {7, 8, 9} 
    // };

    Matrix3d m = Matrix3d::Identity();
    Matrix3d n = Matrix3d::Identity();

    m << 
        1, 2, 3,
        4, 5, 6,
        7, 8, 9;

    n << 
        1, 2, 3,
        4, 5, 6,
        7, 8, 9;

    Matrix3d temp = m * n;


    // matrix_multiply(I33, I33, temp);
    // matrix_add(I33, I33, temp);
    // matrix_transpose(I33, temp);

    return temp;
}

//_____________________________EXTRA STUFF__________________________________________//

unsigned long Estimation::calculate_delta_time()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = (currentTime - oldTime);
  oldTime = currentTime;
  return deltaTime;
}