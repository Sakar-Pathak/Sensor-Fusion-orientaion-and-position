#include "ahrs_ekf.h"

AHRS_EKF::AHRS_EKF(Eigen::Matrix<double, 4, 4> &P_in, Eigen::Matrix<double, 4, 4> &Q_in, Eigen::Matrix<double, 6, 6> &R_in, Eigen::Vector<double, 3> &g_in, Eigen::Vector<double, 3> &r_in, Eigen::Vector<double, 4> &q_in, float &_t)
: Q(Q_in), R(R_in), g(g_in), r(r_in), _t(_t)
{
    P = P_in;
    q = q_in;
}

Eigen::Matrix<double, 4, 4> AHRS_EKF::omega_func(Eigen::Vector<double, 3> &w)
{
    Eigen::Matrix<double, 4, 4> omega;
    omega << 0, -w[0], -w[1], -w[2],
            w[0], 0, w[2], -w[1],
            w[1], -w[2], 0, w[0],
            w[2], w[1], -w[0], 0;
    return omega;
}

Eigen::Matrix<double, 4, 4> AHRS_EKF::F_func(Eigen::Vector<double, 3> &w)
{
    Eigen::Matrix<double, 4, 4> F;
    F = Eigen::Matrix<double, 4, 4>::Identity() + 0.5 * _t * omega_func(w);
    return F;
}

Eigen::Vector<double, 4> AHRS_EKF::f_func(Eigen::Matrix<double, 4, 4> &F)
{
    Eigen::Vector<double, 4> q_;
    q_ = F * q;
    return q_;
}

Eigen::Matrix<double, 3, 4> AHRS_EKF::W_func()
{
    Eigen::Matrix<double, 3, 4> W;
    W << -q[1], -q[2], -q[3],
        q[0], -q[3], q[2],
        q[3], q[0], -q[1],
        -q[2], q[1], q[0];

    return 0.5 * _t * W;
}

// quaternion to rotation matrix
Eigen::Matrix<double, 3, 3> AHRS_EKF::q2R(Eigen::Vector<double, 4> &q)
{
    Eigen::Matrix<double, 3, 3> R;
    R << 1 - 2 * (q[2] * q[2] + q[3] * q[3]), 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2]),
        2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[1] * q[1] - q[3] * q[3]), 2 * (q[2] * q[3] - q[0] * q[1]),
        2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

    return R;
}

Eigen::Vector<double, 6> AHRS_EKF::h_func(Eigen::Vector<double, 4> &q)
{
    Eigen::Vector<double, 6> a_m_;
    a_m_ << q2R(q).transpose() * g, q2R(q).transpose() * r;
    return a_m_;
}

Eigen::Matrix<double, 6, 4> AHRS_EKF::H_func(Eigen::Vector<double, 4> &q)
{
    Eigen::Matrix<double, 6, 4> H;
    H << g[1] * q[3] - g[2] * q[2], g[1] * q[2] + g[2] * q[3], -2 * g[0] * q[2] + g[1] * q[1] - g[2] * q[0], -2 * g[0] * q[3] + g[1] * q[0] + g[2] * q[1],
        -g[0] * q[3] + g[2] * q[1], g[0] * q[2] - 2 * g[1] * q[1] + g[2] * q[0], g[0] * q[1] + g[2] * q[3], -g[0] * q[0] - 2 * g[1] * q[3] + g[2] * q[2],
        g[0] * q[2] - g[1] * q[1], g[0] * q[3] - g[1] * q[0] - 2 * g[2] * q[1], g[0] * q[0] + g[1] * q[3] - 2 * g[2] * q[2], g[0] * q[1] + g[1] * q[2],
        r[1] * q[3] - r[2] * q[2], r[1] * q[2] + r[2] * q[3], -2 * r[0] * q[2] + r[1] * q[1] - r[2] * q[0], -2 * r[0] * q[3] + r[1] * q[0] + r[2] * q[1],
        -r[0] * q[3] + r[2] * q[1], r[0] * q[2] - 2 * r[1] * q[1] + r[2] * q[0], r[0] * q[1] + r[2] * q[3], -r[0] * q[0] - 2 * r[1] * q[3] + r[2] * q[2],
        r[0] * q[2] - r[1] * q[1], r[0] * q[3] - r[1] * q[0] - 2 * r[2] * q[1], r[0] * q[0] + r[1] * q[3] - 2 * r[2] * q[2], r[0] * q[1] + r[1] * q[2];

    return 2 * H;
}

// initial quaternion from acceleration and magnetic field
Eigen::Vector<double, 4> AHRS_EKF::init_quaternion(Eigen::Vector<double, 3> &a, Eigen::Vector<double, 3> &m)
{
    Eigen::Vector<double, 3> v1 = a.cross(m);
    Eigen::Vector<double, 3> v2 = a.cross(v1);
    v1 = v1 / v1.norm();
    v2 = v2 / v2.norm();
    Eigen::Matrix<double, 3, 3> R;
    R << a[0], v1[0], v2[0],
        a[1], v1[1], v2[1],
        a[2], v1[2], v2[2];
        
    Eigen::Quaterniond q_(R);
    q << q_.w(), q_.x(), q_.y(), q_.z();
    return q;
}

Eigen::Vector<double, 4> AHRS_EKF::update(Eigen::Vector<double, 3> &a, Eigen::Vector<double, 3> &m, Eigen::Vector<double, 3> &w)
{
    // predict
    Eigen::MatrixXd(4,4);
    Eigen::Matrix<double, 4, 4> F = F_func(w);
    Eigen::Vector<double, 4> q_ = f_func(F);
    Eigen::Matrix<double, 3, 4> W = W_func();
    Eigen::MatrixXd P_ = F * P * F.transpose() + W * Q * W.transpose();

    // correct
    Eigen::Vector<double, 6> a_m_ = h_func(q_);
    Eigen::Matrix<double, 6, 4> H = H_func(q_);
    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 4, 6> K = P_ * H.transpose() * S.inverse();
    Eigen::Vector<double, 6> z;
    z << a, m;
    q_ = q_ + K * (z - a_m_);
    Eigen::Matrix<double, 4, 4> I = Eigen::Matrix<double, 4, 4>::Identity(4, 4);
    P_ = (I - K * H) * P_;

    // update
    q = q_;
    P = P_;
    return q;
}