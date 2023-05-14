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

// initial quaternion from acceleration and magnetic field
double* init_quaternion(double (&a_in)[3], double (&m_in)[3]);

class AHRS_EKF
{
public:
    AHRS_EKF();
    ~AHRS_EKF();

    void init(const double (&Q_in)[3], const double (&R_in)[6], const double (&g_in)[3], const double(&r_in)[3], const float &Ts, const double (&P_in)[4], const double (&q_in)[4]);

    double* update(double (&a_in)[3], double (&m_in)[3], double (&w_in)[3]);

private:
    Eigen::Matrix<double, 3, 3> Q;  // process noise covariance m
    Eigen::Matrix<double, 6, 6> R;  // measurement noise covariance m
    Eigen::Vector<double, 3> g; // gravity v
    Eigen::Vector<double, 3> r; // magnetic field v
    float _t; // time interval
    
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