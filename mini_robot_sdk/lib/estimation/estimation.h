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
        // VectorXd calibrate_imu();
        // Vector3d get_gyro(Vector3d);
        // Vector3d get_acc();
        // Vector3d get_rpy(VectorXd, Vector3d);
        // Vector3d get_position(Vector3d);
        // VectorXd get_pose();
        VectorXd imu_ekf(Vector3d, Vector3d, Vector3d, Vector3d, VectorXd, Vector3d);
        Vector3d ddr_ekf(float, float, VectorXd);

        // tests - these should be deleted/commented out           
        // MatrixXd imu_process_model(float*, float*);
        // MatrixXd imu_process_noise(float*);
        // MatrixXd imu_measurement_model(float*);

        Matrix3d ddr_process(float, float);
        Matrix3d ddr_measurement_model();
        Vector3d unicycle_model(float, float);
        Vector3d ddr_measurement();

    private:
        double roll0, pitch0, yaw0;
        double phi, theta, psi;
        double phi_gyro, theta_gyro, psi_gyro;

        unsigned long calculate_delta_time();
        VectorXd calib{6};
        Vector3d gyro;
        Vector3d acc;
        Vector3d rpy;
        Vector3d position;
        VectorXd pose{6};

        MatrixXd Fk_imu{12, 12};
        MatrixXd Gk_imu{12, 12};
        VectorXd xk_imu_prev{12};
        VectorXd xk_imu{12};
        MatrixXd P_imu_prev{12,12};
        MatrixXd P_imu{12,12};
        Vector4d zk_imu; 
        MatrixXd Hk_imu{4, 12};
        Matrix4d R_imu;

        MatrixXd imu_process_model(Vector3d, Vector3d);
        MatrixXd imu_process_noise(Vector3d);
        Vector4d imu_measurement(float, Vector3d, Vector3d, VectorXd, Vector3d);
        MatrixXd imu_measurement_model(Vector3d);
        void imu_predict(VectorXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd);
        void imu_update(VectorXd, VectorXd, MatrixXd, Vector3d, Vector3d, Vector3d, VectorXd, Vector3d);
        

        Matrix3d Fk_ddr;
        Vector3d f_ddr{f_ddr[0], f_ddr[1], f_ddr[2]}; 
        Vector3d xk_ddr_prev{xk_ddr_prev[0], xk_ddr_prev[1], xk_ddr_prev[2]};
        Vector3d xk_ddr;
        Matrix3d P_ddr_prev;
        Matrix3d P_ddr;
        Matrix3d Qk_ddr;
        Vector3d zk_ddr{zk_ddr[0], zk_ddr[1], zk_ddr[2]}; 
        Matrix3d Hk_ddr;
        Matrix3d R_ddr;

        // Vector3d unicycle_model(float, float);
        // Matrix3d ddr_process(float v, float heading);
        // Vector3d ddr_measurement(float*);
        // Matrix3d ddr_measurement_model();
        void ddr_predict(Matrix3d, float, float, VectorXd);
        void ddr_update(Vector3d, Vector3d, Matrix3d);
};

#endif