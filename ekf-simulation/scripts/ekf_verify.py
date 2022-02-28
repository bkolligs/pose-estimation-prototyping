import numpy as np
import matplotlib.pyplot as plt
import moonranger as m
from termcolor import colored

from ParseRawIMUData import parseIMUFile
from noisy_rover import NoisyRover

if __name__ == '__main__':
    rover = NoisyRover(sigma_gyro=0.5)
    rover.ekf.set_imu_count(500)

    print("Performing test")
    time, gyro, accel, inclin = parseIMUFile("../imu_data_logs/imu_data4.txt")

    num_datapoints = len(time)

    for idx, (timestamp, gyro_vals, accel_val, inclin_vals) in enumerate(zip(time, gyro, accel, inclin)):
        rover.simulate(gyro_sensor=gyro_vals, acc_sensor=accel_val)
        print(f"Time: {100 * idx / num_datapoints}", end="\r")
        
        rover.plot()

    print(colored("\nSimulation done!", 'green'))
    plt.show()
