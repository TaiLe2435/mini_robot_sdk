// estimation.h
#ifndef estimation_h
#define estimation_h

#include <Arduino.h>

class Estimation {
    public:
        Estimation();
        float* calib_gyro();
        float* get_gyro(float* gyro_calib);
        float* get_acc();
        // float* get_rpy();
        void imu_ekf();
        void ddr_ekf();
        typedef float (*Matrix12x12Pointer)[12];
        Matrix12x12Pointer imu_process(float* rpy, float* acc);
    private:
        float phi, theta, psi;
        float phi_0, theta_0, psi_0, psi_IC;
        float psiOffset;

        float calib[3];
        float gyro[3];
        float acc[3];

        float Fk[12][12];
        // typedef float (*Matrix12x12Pointer)[12];

        // Matrix12x12Pointer imu_process(float* rpy, float* acc);
        void imu_measurement();
        void imu_predict();
        void imu_update();
        
        void ddr_process();
        void ddr_measurement();
        void ddr_predict();
        void ddr_update();

        void insert_matrix(float largeMatrix[12][12], float smallMatrix[3][3], int row, int col);
};

#endif