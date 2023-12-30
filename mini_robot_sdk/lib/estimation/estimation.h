// estimation.h
#ifndef estimation_h
#define estimation_h

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;
class Estimation {
    public:
        Estimation();
        float* calib_gyro();
        float* get_gyro(float*);
        float* get_acc();
        // float* get_rpy();
        void imu_ekf();
        void ddr_ekf();

        // tests - these should be deleted/commented out
        typedef float (*MatrixNx12Pointer)[12];
           
        MatrixXd imu_process_model(float*, float*);
        MatrixXd imu_process_noise(float*);
        MatrixXd imu_measurement_model(float*);

        typedef float (*MatrixNx3Pointer)[3];
        MatrixNx3Pointer ddr_process(float, float);
        MatrixNx3Pointer ddr_measurement_model();

        Matrix3d matrix_test();

    private:
        float phi, theta, psi;
        float phi_0, theta_0, psi_0, psi_IC;
        float psiOffset;

        unsigned long calculate_delta_time();
        float calib[3];
        float gyro[3];
        float acc[3];

        MatrixXd Fk_imu{12, 12};
        MatrixXd Gk_imu{12, 12};
        float xk_imu[12];
        float zk_imu[4]; 
        MatrixXd Hk_imu{4, 12};
        // typedef float (*MatrixNx12Pointer)[12];

        // MatrixNx12Pointer imu_process_model(float*, float*);
        // MatrixNx12Pointer imu_process_noise(float*);
        float* imu_measurement(float, float*, float*);
        // MatrixNx12Pointer imu_measurement_model(float*);
        void imu_predict();
        void imu_update();
        

        float Fk_ddr[3][3];
        float f_ddr[3]; 
        float xk_ddr_prev[3];
        float xk_ddr[3];
        float P_ddr_prev[3][3];
        float P_ddr[3][3];
        float Qk_ddr[3][3];
        float zk_ddr[3]; 
        float Hk_ddr[3][3];
        float R_ddr[3][3];
        float* unicycle_model(float, float);
        // MatrixNx3Pointer ddr_process(float v, float heading);
        float* ddr_measurement(float*);
        // MatrixNx3Pointer ddr_measurement_model();
        void ddr_predict(float[][3], float[][3], float, float);
        void ddr_update(float*, float*, float[][3]);

        // Matrix stuff
        void insert_matrix_Nx12(float[][12], float[][3], int, int, int, int, int, int);
        void insert_matrix_Nx12_default(float[][12], float[][3], int, int);
        void insert_matrix_Nx3(float[][3], float[][3], int, int);
        void matrix_multiply(float[][3], float[][3], float[][3]);
        void matrix_add(float[][3], float[][3], float[][3]);
        void matrix_transpose(float[][3], float[][3]);
        float temp[3][3];
};

#endif