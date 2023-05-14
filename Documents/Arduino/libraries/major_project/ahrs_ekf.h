#ifndef AHRS_EKF_H
#define AHRS_EKF_H
#if defined(ARDUINO)
  #include "eigen.h"
  #include <Eigen/Dense>
#else
  #include <sys/time.h>
  #include <stdint.h>
  #include <math.h>
  #include <Eigen/Core>
  #include <Eigen/Dense>

#endif

class AHRS_EKF
{
public:
    AHRS_EKF(Eigen::Matrix<double, 3, 3> &Q_in, Eigen::Matrix<double, 6, 6> &R_in, Eigen::Vector<double, 3> &g_in, Eigen::Vector<double, 3> &r_in, float &Fs, Eigen::Matrix<double, 4, 4> &P_in, Eigen::Vector<double, 4> &q_in);
    ~AHRS_EKF();

    // initial quaternion from acceleration and magnetic field
    Eigen::Vector<double, 4> init_quaternion(Eigen::Vector<double, 3> &a, Eigen::Vector<double, 3> &m);

    Eigen::Vector<double, 4> update(Eigen::Vector<double, 3> &a, Eigen::Vector<double, 3> &m, Eigen::Vector<double, 3> &w);

private:
    const Eigen::Matrix<double, 3, 3> Q;  // process noise covariance matrix

    const Eigen::Matrix<double, 6, 6> R;  // measurement noise covariance matrix

    const Eigen::Vector<double, 3> g; // gravity vector

    const Eigen::Vector<double, 3> r; // magnetic field vector

    const float _t; // time interval

    Eigen::Vector<double, 4> q; // quaternion at time t-1

    Eigen::Matrix<double, 4, 4> P; // state covariance matrix at time t-1


    Eigen::Matrix<double, 4, 4> omega_func(Eigen::Vector<double, 3> &w);

    Eigen::Matrix<double, 4, 4> F_func(Eigen::Vector<double, 3> &w);

    Eigen::Vector<double, 4> f_func(Eigen::Matrix<double, 4, 4> &F);

    Eigen::Matrix<double, 4, 3> W_func();

    // quaternion to rotation matrix
    Eigen::Matrix<double, 3, 3> q2R(Eigen::Vector<double, 4> &q);

    Eigen::Vector<double, 6> h_func(Eigen::Vector<double, 4> &q);

    Eigen::Matrix<double, 6, 4> H_func(Eigen::Vector<double, 4> &q);

};
#endif