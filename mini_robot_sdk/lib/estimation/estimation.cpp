#include "estimation.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h> // I2C lib
#include <LSM6.h> // Accel and Gyro lib
#include <LIS3MDL.h> // Magnetometer lib

LSM6 gyroAcc; 
LIS3MDL mag;

unsigned long oldTime {0}; // used to calculate time step

Estimation::Estimation(){
    
}

unsigned long Estimation::calculate_delta_time()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = (currentTime - oldTime);
  oldTime = currentTime;
  return deltaTime;
}

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

// float* Estimation::get_rpy(){
// //     return 0.0;
// }

void Estimation::insert_matrix(float largeMatrix[][12], float smallMatrix[][3], int numRowsLg, int numColsLg, int numRowsSm, int numColsSm, int row, int col) {
    for (int i = 0; i < numRowsSm; ++i) {
        for (int j = 0; j < numColsSm; ++j) {
            largeMatrix[row + i][col + j] = smallMatrix[i][j];
        }
    }
}

void Estimation::insert_matrix_default(float largeMatrix[][12], float smallMatrix[][3], int row, int col) {
    insert_matrix(largeMatrix, smallMatrix, 12, 12, 3, 3, row, col);
}

Estimation::Matrix12x12Pointer Estimation::imu_process(float* rpy, float* acc){
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

    insert_matrix_default(Fk, zero33, 0, 0);
    insert_matrix_default(Fk, zero33, 0, 3);
    insert_matrix_default(Fk, neg_C, 0, 6);
    insert_matrix_default(Fk, zero33, 0, 9);

    insert_matrix_default(Fk, S, 3, 0);
    insert_matrix_default(Fk, zero33, 3, 3);
    insert_matrix_default(Fk, zero33, 3, 6);
    insert_matrix_default(Fk, C, 3, 9);

    insert_matrix_default(Fk, zero33, 6, 0);
    insert_matrix_default(Fk, zero33, 6, 3);
    insert_matrix_default(Fk, betaGI33, 6, 6);
    insert_matrix_default(Fk, zero33, 6, 9);

    insert_matrix_default(Fk, zero33, 9, 0);
    insert_matrix_default(Fk, zero33, 9, 3);
    insert_matrix_default(Fk, zero33, 9, 6);
    insert_matrix_default(Fk, betaAI33, 9, 9);

    return Fk;
}

Estimation::Matrix12x12Pointer Estimation::imu_process_noise(float* rpy){

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

    insert_matrix_default(Gk, neg_C, 0, 0);
    insert_matrix_default(Gk, zero33, 0, 3);
    insert_matrix_default(Gk, zero33, 0, 6);
    insert_matrix_default(Gk, zero33, 0, 9);

    insert_matrix_default(Gk, zero33, 3, 0);
    insert_matrix_default(Gk,C, 3, 3);
    insert_matrix_default(Gk, zero33, 3, 6);
    insert_matrix_default(Gk, zero33, 3, 9);

    insert_matrix_default(Gk, zero33, 6, 0);
    insert_matrix_default(Gk, zero33, 6, 3);
    insert_matrix_default(Gk, I33, 6, 6);
    insert_matrix_default(Gk, zero33, 6, 9);

    insert_matrix_default(Gk, zero33, 9, 0);
    insert_matrix_default(Gk, zero33, 9, 3);
    insert_matrix_default(Gk, zero33, 9, 6);
    insert_matrix_default(Gk, I33, 9, 9);

    return Gk;
}

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

void Estimation::imu_predict(){

}

void Estimation::imu_update(){
    
}

void Estimation::imu_ekf(){

}

void Estimation::ddr_process(){

}

void Estimation::ddr_measurement(){
    
}

void Estimation::ddr_predict(){

}

void Estimation::ddr_update(){
    
}

void Estimation::ddr_ekf(){

}