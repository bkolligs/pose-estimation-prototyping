import moonranger as m
import numpy as np
import matplotlib.pyplot as plt
from termcolor import colored
from noisy_rover import NoisyRover

if __name__ == '__main__':
	rover = NoisyRover(sigma_imu=0.5)
	ekf = m.OrientationEKF()
	ekf.set_imu_count(500)

	print(f"Starting EKF as Initialized: {ekf.is_ekf_initialized()}")
	for i in range(100):
		# Generate some noisy data
		imu_sensor, acc_sensor, incl_sensor = rover.noisy_data()
		ekf.handle_imu(imu_sensor, acc_sensor)
		print(f"State: {ekf.get_state()}", end='\r')

		rover.plot()
	
	print(colored("\nSimulation done!", 'green'))
	plt.show()
