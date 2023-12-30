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
        VectorXd imu_ekf(float*, float*, float*, float*);
        Vector3d ddr_ekf(float, float, float*);

        // tests - these should be deleted/commented out           
        // MatrixXd imu_process_model(float*, float*);
        // MatrixXd imu_process_noise(float*);
        // MatrixXd imu_measurement_model(float*);

        Matrix3d ddr_process(float, float);
        Matrix3d ddr_measurement_model();
        Vector3d unicycle_model(float, float);
        Vector3d ddr_measurement(float*);

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
        VectorXd xk_imu_prev{12};
        VectorXd xk_imu{12};
        MatrixXd P_imu_prev{12,12};
        MatrixXd P_imu{12,12};
        Vector4d zk_imu; 
        MatrixXd Hk_imu{4, 12};
        Matrix4d R_imu;

        MatrixXd imu_process_model(float*, float*);
        MatrixXd imu_process_noise(float*);
        Vector4d imu_measurement(float, float*, float*);
        MatrixXd imu_measurement_model(float*);
        void imu_predict(VectorXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd);
        void imu_update(VectorXd, VectorXd, MatrixXd, float*, float*, float*);
        

        Matrix3d Fk_ddr;
        Vector3d f_ddr{f_ddr[0], f_ddr[1], f_ddr[2]}; 
        Vector3d xk_ddr_prev{xk_ddr_prev[0], xk_ddr_prev[1], xk_ddr_prev[2]};
        Vector3d xk_ddr;
        Matrix3d P_ddr_prev;
        Matrix3d P_ddr;
        Matrix2d Qk_ddr;
        Vector3d zk_ddr{zk_ddr[0], zk_ddr[1], zk_ddr[2]}; 
        Matrix3d Hk_ddr;
        Matrix3d R_ddr;

        // Vector3d unicycle_model(float, float);
        // MatrixNx3Pointer ddr_process(float v, float heading);
        // Vector3d ddr_measurement(float*);
        // MatrixNx3Pointer ddr_measurement_model();
        void ddr_predict(Matrix3d, float, float, float*);
        void ddr_update(Vector3d, Vector3d, Matrix3d);
};

#endif