#include <iostream>

#include <math.h>

#include "Eigen/Core"
#include "Eigen/Dense"


class EKF {
private:
    Eigen::MatrixXd P_; // state covariance matrix

    Eigen::MatrixXd Q;  // process noise covariance matrix
    Eigen::MatrixXd H;  // measurement matrix
    Eigen::MatrixXd R;  // measurement noise covariance matrix

    float _t;  // time between samples in seconds



public:
    EKF(Eigen::MatrixXd &P_in, Eigen::MatrixXd &Q_in, Eigen::MatrixXd &R_in, float &Fs);
    ~EKF();
    void predict(Eigen::MatrixXd &F, Eigen::MatrixXd &L);

    void correct(Eigen::VectorXd &x_, Eigen::VectorXd &y_, Eigen::VectorXd &z, Eigen::MatrixXd &H, Eigen::MatrixXd &M); // x_ is the predicted state
}