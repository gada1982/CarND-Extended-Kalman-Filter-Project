#include "kalman_filter.h"
#include "math.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    
    UpdateFunc(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    
    // Calculate ro
    double ro = sqrt(pow(px, 2) + pow(py, 2));
    
    // Calculate phi -> check size
    double phi = .001;
    if(fabs(px) > .001) {
        phi = atan2(py, px);
    }
    
    // Calculate ro_d -> check size
    double dif;
    
    // Check size
    if (ro < .001){
        dif = .001;
        
    }
    else {
        dif = ro;
    }
    
    double ro_d = (px * vx + py * vy) / dif;
    
    VectorXd hx = VectorXd(3);
    hx << ro, phi, ro_d;
    
    // prediction error
    VectorXd y = z - hx;
    
    // Keep within -PI and +PI
    while(y[1] < -M_PI || y[1] > M_PI)
    {
        if(y[1] > M_PI) {
            y[1] =  y[1] - 2 * M_PI;
        }
        else {
            y[1] = y[1] + 2 * M_PI;
        }
    }
    
    UpdateFunc(y);
}

void KalmanFilter::UpdateFunc(const VectorXd &y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

