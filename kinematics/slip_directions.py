import numpy as np
import matplotlib.pyplot as plt
from four_wheel.four_wheel import FourWheel
from four_wheel.utils import drive_arc_convert

if __name__ == '__main__':
	rover = FourWheel(0.32195, 0.2222, 0.0745, 0.0955)
	# adjust the rover slip constraints in x and y
	rover.v_constraints[[0, 6]] = 0.05
	rover.v_constraints[[3, 9]] = 0.02
	wheels = rover.actuation([0, 0, 0.5, 0, 0, 0])
	# print(wheels)
	body = rover.navigation([0.05, 0.05, 0.05, 0.05])
	print(body)