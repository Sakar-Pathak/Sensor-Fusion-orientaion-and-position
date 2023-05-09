import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
import matplotlib.pyplot as plt

print("\n\n\n\n")

# Load data from csv file
data = np.genfromtxt('..\lovegarden_rawdata_2.csv', delimiter=',')

# Load acceleromter data
acc_data = data[:, 5:8]

# normalize accelerometer data
acc = acc_data / np.linalg.norm(acc_data, axis=1, keepdims=True)

# Load gyroscope data
gyr = data[:, 8:11]

# Load magnetometer data
mag_data = data[:, 11:14]

# normalize magnetometer data
mag = mag_data / np.linalg.norm(mag_data, axis=1, keepdims=True)

calib_data_length = 1600

# angular velocity already in rad/sec

# finding covariance of gyroscope data up to calibration data length
gyr_cov = np.mean(np.diagonal(np.cov(gyr[0:calib_data_length, :], rowvar=False)))

# finding covariance of accelerometer data up to calibration data length
acc_cov = np.mean(np.diagonal(np.cov(acc[0:calib_data_length, :], rowvar=False)))

# finding covariance of magnetometer data up to calibration data length
mag_cov = np.mean(np.diagonal(np.cov(mag[0:calib_data_length, :], rowvar=False)))


# finding mean of magnetometer data up to calibration data length
mag_mean = np.mean(mag[0:calib_data_length, :], axis=0)

# finding mean of accelerometer data up to calibration data length
acc_mean = np.mean(acc[0:calib_data_length, :], axis=0)


# finding initial quaternion
q0 = acc2q(acc_mean)


# Run EKF algorithm
ekf = EKF(gyr=gyr, acc=acc, mag=mag, frequency=250.0, magnetic_ref = mag_mean, q0 = q0, noises=(gyr_cov, acc_cov, mag_cov))


# Plot using matplotlib
fig, ax = plt.subplots()
ax.plot(ekf.Q[:, 0], label='q0')
ax.plot(ekf.Q[:, 1], label='q1')
ax.plot(ekf.Q[:, 2], label='q2')
ax.plot(ekf.Q[:, 3], label='q3')
ax.set_xlabel('Time [s]')
ax.set_ylabel('quaternion')
ax.legend()
plt.show()

print("\n\n\n\n")