#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
    /**
     * Constructor.
     */
    FusionEKF();
    
    /**
     * Destructor.
     */
    virtual ~FusionEKF();
    
    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    
    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;
    
private:
    // check whether the tracking toolbox was initiallized or not (first measurement)
    bool is_initialized_;
    
    // previous timestamp
    long long previous_timestamp_;
    
    // tool object used to compute Jacobian and RMSE
    Tools tools;
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
    
    // Process Noise
    float noise_ax;
    float noise_ay;
    
    // Configure which sensor should be used (Rader, Laser or both)
    // This is for testing the possible accurancy with only one sensor-type used
    // For the final testing both sensor have to be used, because this gives the best result
    
    // Use LIDAR/Laser Data
    bool useLaser;
    
    // Use RADAR Data
    bool useRadar;

};

#endif /* FusionEKF_H_ */
