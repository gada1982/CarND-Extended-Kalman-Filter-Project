#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    // Measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    // Measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    
    // Measurement matrix - laser
    H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
    
    // initialize jacobian matrix - radar
    Hj_ << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
    
    // Set the process and measurement noise
    noise_ax = 9.0;
    noise_ay = 9.0;
    
    // Configure which sensor should be used (Rader, Laser or both)
    // This is for testing the possible accurancy with only one sensor-type used
    // For the final testing both sensor have to be used, because this gives the best result
    
    // Use LIDAR/Laser Data
    useLaser = true;
    
    // Use RADAR Data
    useRadar = true;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // Initialize the state ekf_.x_
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        
        // State covariance matrix P
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
        
        // Initial transition matrix F_
        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && useRadar == true) {
            // Convert radar from polar to cartesian coordinates and initialize state.
            
            double ro = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double ro_d = measurement_pack.raw_measurements_[2];
            
            double px = ro * cos(phi);
            double py = ro * sin(phi);
            double vx = ro_d * cos(phi);
            double vy = ro_d * sin(phi);
            
            ekf_.x_ << px, py, vx, vy;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && useLaser == true) {
            //set the state with the initial location and zero velocity
            double px = measurement_pack.raw_measurements_[0];
            double py = measurement_pack.raw_measurements_[1];
            double vx = 0.0;
            double vy = 0.0;
            
            ekf_.x_ << px, py, vx, vy;
        }
        else {
            // Can't initialize measurement data
            // Measurement type and used filter are not equivalent
            // Check FusionEKF::FusionEKF() for configuration
            // Both sensor types are disabled if you get here
            return;
        }
        
        previous_timestamp_ = measurement_pack.timestamp_;
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    cout << "\ndt = " << dt << endl;
    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    
    // Set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
    0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
    dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
    // Call the Extemded Kalman Filter predict() function
    
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update the state and covariance matrices (depending on the sensor type)
     ****************************************************************************/
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && useRadar == true) {
        // Radar updates
        Tools tools;
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && useLaser == true) {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    else {
        // Nothing to update
        // Measurement type and used filter are not equivalent
        // Check FusionEKF::FusionEKF() for configuration
        // Both sensor types are disabled if you get here
    }
    
    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
