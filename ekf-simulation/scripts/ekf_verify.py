import numpy as np
import matplotlib.pyplot as plt
import moonranger as m
from termcolor import colored

from ParseRawIMUData import parseIMUFile
from noisy_rover import NoisyRover

if __name__ == '__main__':
    rover = NoisyRover(sigma_gyro=0.5)
    ekf = m.OrientationEKF()
    ekf.set_imu_count(500)
    for i in range(100):
        # Generate some noisy data
        rover.simulate()
        rover.plot()

	print("Performing test")
    time, gyro, accel, inclin = parseIMUFile("../imu_data_logs/imu_data4.txt")
    rover = NoisyRover(sigma_imu=0.5)


    num_datapoints = len(time)

    for idx, (timestamp, gyro_vals, accel_val, inclin_vals) in enumerate(zip(time, gyro, accel, inclin)):
        ekf.handle_imu(gyro_vals, accel_val)
        # print(f"State: {ekf.get_state()}")
        print(100 * idx / num_datapoints)

        rover.plot()

    print(colored("\nSimulation done!", 'green'))
    plt.show()
