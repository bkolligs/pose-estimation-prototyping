import numpy as np
import robotics as r
from matplotlib import pyplot as plt


class NoisyRover:
    """
    This class represents the sensor measurements for a rover that we can use for testing the EKF
    """

    def __init__(self, initial_position=[0, 0, 0], initial_ori=[0, 0, 0], sigma_gyro=0.1, sigma_acc=0.1, sigma_incl=0.1,
                 delta_t=0.01) -> None:
        self.sigma_gyro, self.sigma_acc, self.sigma_incl = sigma_gyro, sigma_acc, sigma_incl
        self.delta_t = delta_t

        # Establish the intitial pose
        self.pose = r.Transform(
            x=initial_position[0],
            y=initial_position[1],
            z=initial_position[2],
            theta=initial_ori[0],
            phi=initial_ori[1],
            psi=initial_ori[2]
        )

    def noisy_data(self, update_self=True):
        """
        Generate noisy data from the IMU
        """
        gyro_noisy = np.random.normal(scale=self.sigma_gyro, size=3)
        acc_noisy = np.random.normal(scale=self.sigma_acc, size=3)
        incl_noisy = np.random.normal(scale=self.sigma_incl, size=3)

        if update_self: self.pose.update_transform(
            theta=gyro_noisy[0] * self.delta_t,
            phi=gyro_noisy[1] * self.delta_t,
            psi=gyro_noisy[2] * self.delta_t,
        )
        return gyro_noisy, acc_noisy, incl_noisy

    def plot(self):
        plt.clf()
        # Plot the rover
        axis = plt.axes(projection='3d')
        self.pose.plot(detached=True, axis_obj=axis)
        xlim, ylim, zlim = 1.5, 1.5, 1.5
        axis.set_xlim(-xlim, xlim)
        axis.set_ylim(-ylim, ylim)
        axis.set_zlim(-zlim, zlim)
        plt.draw()
        plt.pause(0.01)
