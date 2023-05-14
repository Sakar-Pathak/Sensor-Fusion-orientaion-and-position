#include "ahrs_ekf.h"

#include "MPU9250.h"



#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET 0.0000702
#define GYRO_Y_OFFSET -0.0001424
#define GYRO_Z_OFFSET -0.0003332

#define ACCEL_X_OFFSET -1.4086211
#define ACCEL_Y_OFFSET -8.8509712
#define ACCEL_Z_OFFSET -9.3008728

const float magn_ellipsoid_center[3] = {33.0393, 27.8563, -6.24899};
const float magn_ellipsoid_transform[3][3] = {{0.843576, -0.0375454, -0.0320949}, {-0.0375454, 0.838998, 0.0437144}, {-0.0320949, 0.0437144, 0.97602}};


const double g[3] = {0, 0, 1};
//const float r[3] = {0, 0, 0};
const float Ts = 0.01;
const double P[4] = {1,1,1,1}; //uncertainty of quaternion
//const float q[4] = {1,0,0,0}; //initial quaternion

// Covariance matrices
const double Q[3] = {0.0001, 0.0001, 0.0001};  // covariance for gyro

const double R[6] = {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001};  // covariance for accel and mag


//extern double accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//extern double magnetom[3];
//extern double gyro[3];


extern double* q;

void imu_read();
void compensate_imu_errors();
void normalize_AM();
void Matrix_Vector_Multiply(const float a[3][3], const double b[3], double out[3]);
void imu_setup();
void ahrs_setup();
void ahrs_update();