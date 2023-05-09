import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
#import matplotlib.pyplot as plt

from panda3d.core import loadPrcFile    #to load .prc file
loadPrcFile("config\config.prc")        #it contains configurational settings

from panda3d.core import NodePath, Loader, Quat
from direct.showbase.ShowBase import ShowBase


print("\n\n\n\n")

# Load data from csv file
data = np.genfromtxt('..\lovegarden_rawdata_2.csv', delimiter=',')

# Load madgwick quaternion data with beta = 0.3
madgwick = data[:, 1:5]

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
#ekf = EKF(gyr=gyr, acc=acc, mag=mag, frequency=250.0, magnetic_ref = mag_mean, q0 = q0, noises=(gyr_cov, acc_cov, mag_cov))


"""
# Plot using matplotlib
fig, ax = plt.subplots()
ax.plot(ekf.Q[:, 0], label='q0')
ax.plot(ekf.Q[:, 1], label='q1')
ax.plot(ekf.Q[:, 2], label='q2')
ax.plot(ekf.Q[:, 3], label='q3')
ax.set_xlabel('Time [s]')ekf
ax.set_ylabel('quaternion')
ax.legend()
plt.show()
"""

class Game(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.model = self.loader.loadModel("models/misc/rgbCube")
        self.model.reparentTo(render)
        self.model.setPos(0, 100, 0)
        self.model.setScale(20,20,5)

        self.taskMgr.add(self.rotate_model_task, "rotate_model_task")

        # Set up the initial rotation
        initial_quat = Quat()
      
        """
        initial_quat.setW(ekf.Q[0, 0])
        initial_quat.setX(-ekf.Q[0, 3])
        initial_quat.setY(-ekf.Q[0, 1])
        initial_quat.setZ(ekf.Q[0, 2])
        """
        """
        initial_quat.setW(1/np.sqrt(2))
        initial_quat.setX(0)
        initial_quat.setY(1/np.sqrt(2))
        initial_quat.setZ(0)
        """

        #madgwick quaternion visualize
        initial_quat.setW(madgwick[0, 0])
        initial_quat.setX(-madgwick[0, 3])
        initial_quat.setY(-madgwick[0, 1])
        initial_quat.setZ(madgwick[0, 2])


        self.model.setQuat(initial_quat)

        self.gap = 250/60       # 60 fps is the frame rate and 250 is the frequency of the data
        self.i = 0



    def rotate_model_task(self, task):
        
        self.i = self.i + self.gap
        int_i = int(self.i)
        #if(int_i<len(ekf.Q)): 
        if(int_i<len(madgwick)):
            quat = Quat()
            """
            quat.setW(ekf.Q[int_i, 0])
            quat.setX(-ekf.Q[int_i, 3])
            quat.setY(-ekf.Q[int_i, 1])
            quat.setZ(ekf.Q[int_i, 2])
            """
            # madgwick quaternion visualize
            quat.setW(madgwick[int_i, 0])
            quat.setX(-madgwick[int_i, 3])
            quat.setY(-madgwick[int_i, 1])
            quat.setZ(madgwick[int_i, 2])


            self.model.setQuat(quat)
        
        return task.cont


game = Game()
game.run()

print("\n\n\n\n")