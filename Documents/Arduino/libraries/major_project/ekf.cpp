
#include "ekf.h"

EKF::EKF(Eigen::MatrixXd &P_in, Eigen::MatrixXd &Q_in, Eigen::MatrixXd &R_in, float &Fs)
{
    P_ = P_in;
    Q = Q_in;
    R = R_in;
    _t = 1.0/Fs;
}

EKF::~EKF() {}

void EKF::predict(Eigen::MatrixXd &F, Eigen::MatrixXd &L)
{
    // predict the state covariance
    P_ = F * P_ * F.transpose() + L * Q * L.transpose();
}

void EKF::correct(Eigen::VectorXd &x_, Eigen::VectorXd &y_, Eigen::VectorXd &z, Eigen::MatrixXd &H, Eigen::MatrixXd &M) // x_ is the predicted state
{
        // difference between predicted and measured outputs
        Eigen::VectorXd v = z - y_;       // z -> measured output, y_in -> predicted output


        // measurement update
        Eigen::MatrixXd S = H * P_ * H.transpose() + M * R * M.transpose();
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());

        // update the state by using Kalman Filter equations
        x_ = x_ + K * v;            // x_ -> updated state
        P_ = (I - K * H) * P_;
}