//#include <math.h>

#include "Eigen/Core"
#include "Eigen/Dense"

class AHRS_EKF
{
private:
    Eigen::Vector<double, 4> q; // quaternion at time t-1

    Eigen::Matrix<double, 4, 4> P; // state covariance matrix at time t-1

    const Eigen::Matrix<double, 4, 4> Q;  // process noise covariance matrix

    const Eigen::Matrix<double, 6, 6> R;  // measurement noise covariance matrix

    const Eigen::Vector<double, 3> g; // gravity vector

    const Eigen::Vector<double, 3> r; // magnetic field vector

    const float _t; // time interval

public:
    AHRS_EKF(Eigen::Matrix<double, 4, 4> &P_in, Eigen::Matrix<double, 4, 4> &Q_in, Eigen::Matrix<double, 6, 6> &R_in, Eigen::Vector<double, 3> &g_in, Eigen::Vector<double, 3> &r_in, Eigen::Vector<double, 4> &q_in, float &_t);
    ~AHRS_EKF();

    Eigen::Matrix<double, 4, 4> omega_func(Eigen::Vector<double, 3> &w);

    Eigen::Matrix<double, 4, 4> F_func(Eigen::Vector<double, 3> &w);

    Eigen::Vector<double, 4> f_func(Eigen::Matrix<double, 4, 4> &F);

    Eigen::Matrix<double, 3, 4> W_func();

    // quaternion to rotation matrix
    Eigen::Matrix<double, 3, 3> q2R(Eigen::Vector<double, 4> &q);

    Eigen::Vector<double, 6> h_func(Eigen::Vector<double, 4> &q);

    Eigen::Matrix<double, 6, 4> H_func(Eigen::Vector<double, 4> &q);

    // initial quaternion from acceleration and magnetic field
    Eigen::Vector<double, 4> init_quaternion(Eigen::Vector<double, 3> &a, Eigen::Vector<double, 3> &m);

    Eigen::Vector<double, 4> update(Eigen::Vector<double, 3> &a, Eigen::Vector<double, 3> &m, Eigen::Vector<double, 3> &w);

};
