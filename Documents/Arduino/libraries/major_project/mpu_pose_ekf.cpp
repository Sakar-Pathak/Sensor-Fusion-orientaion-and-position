#include "mpu_pose_ekf.h"

#include "Arduino.h"

// Sensor variables
double accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
double magnetom[3];
double gyro[3];

double* q;
double* x;

double bmp_altitude_prev;

uint8_t counter = 25;

MPU9250 IMU(Wire,0x68);

AHRS_EKF ahrs_ekf;
POS_EKF pos_ekf;

unsigned long iTOW_prev = 0;

// Read data from MPU9250
void imu_read() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();

  magnetom[0] = IMU.getMagX_uT();
  magnetom[1] = IMU.getMagY_uT();
  magnetom[2] = IMU.getMagZ_uT();
    
  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}

// Apply calibration to raw sensor readings
void compensate_imu_errors() {
    // Compensate accelerometer error
    accel[0] = accel[0] - ACCEL_X_OFFSET;
    accel[1] = accel[1] - ACCEL_Y_OFFSET;
    accel[2] = accel[2] - (ACCEL_Z_OFFSET + GRAVITY);

    double magnetom_tmp[3];

    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

    // Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET;
    gyro[1] -= GYRO_Y_OFFSET;
    gyro[2] -= GYRO_Z_OFFSET;
}


// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const double b[3], double out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}


void imu_setup()
{
  
  IMU.begin();

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);
}

void pose_setup()
{
  imu_setup();
  gps_setup();
  bmp_setup();

  double accel_avg[3] = {0, 0, 0};
  double magnetom_avg[3] = {0, 0, 0};
  // read average accel and magnetom value for some time to estimate initial quaternion
  for (int i = 0; i < 100; i++)
  {
    imu_read();
    compensate_imu_errors();
    for (int j = 0; j < 3; j++)
    {
      accel_avg[j] += accel[j];
      magnetom_avg[j] += magnetom[j];
    }
    delay(10);
  }
  for (int j = 0; j < 3; j++)
  {
    accel_avg[j] /= 100;
    magnetom_avg[j] /= 100;
  }
  
  // initial quaternion from acceleration and magnetic field
  q = init_quaternion(accel_avg, magnetom_avg);

  const double q_temp[4] = {q[0], q[1], q[2], q[3]};
  //const double q_temp[4] = {1, 0, 0, 0};
  ahrs_ekf.init(Q_ahrs, R_ahrs, g, r, Ts, P_ahrs, q_temp);


  // average gps reading for some time to estimate initial position
  double pos_ref_lla[3] = {0, 0, 0};
  gps_read();
  iTOW_prev = iTOW;
  for (int i = 0; i < 100; i++)
  {
    while(iTOW == iTOW_prev)
    {
        gps_read();
    }
    iTOW_prev = iTOW;
    bmp_read();
    pos_ref_lla[0] += latitude;
    pos_ref_lla[1] += longitude;
    pos_ref_lla[2] += bmp_altitude;
  }
  for (int i = 0; i < 3; i++)
    pos_ref_lla[i] /= 100;

  pos_ref_lla[0] *= PI/180;
  pos_ref_lla[1] *= PI/180;

  // initial state vector
  double x_init[9] = {0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0};
  
  double g_mss[3] = {g[0], g[1], g[2]*GRAVITY};

  pos_ekf.init(Q_pos, R_pos, g_mss, pos_ref_lla, Ts, P_pos, x_init);

  bmp_altitude_prev = bmp_altitude;
}

void pose_update()
{
  imu_read();

  compensate_imu_errors();

  // update
  q = ahrs_ekf.update(accel, magnetom, gyro);

  // update pos
  double u[7] = {q[0], q[1], q[2], q[3], accel[0], accel[1], accel[2]};

  double bmp_vel = 0;
  gps_read();
  if (iTOW - iTOW_prev > 0)
  {
    bmp_read();
    bmp_vel = (bmp_altitude - bmp_altitude_prev)/0.1;  //GPS update rate 0.1s
    bmp_altitude_prev = bmp_altitude;
    iTOW_prev = iTOW;
  }

  double measurement[6] = {velN, velE, bmp_vel, latitude*PI/180, longitude*PI/180,bmp_altitude};
  
  x = pos_ekf.update(u, ahrs_ekf.P, iTOW, measurement);
}