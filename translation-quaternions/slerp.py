import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

class SLERPTest:
	def __init__(self, initial_ori: list, initial_position: list) -> None:
		self.ori = self.__convert_from_euler_angles_to_quat(initial_ori[0], initial_ori[1], initial_ori[2])
		self.position = np.array(initial_position)

	def __convert_from_euler_angles_to_quat(self, roll, pitch, yaw):
		'''
		https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		'''
		cr = np.cos(roll*0.5)
		sr = np.sin(roll*0.5)
		cp = np.cos(pitch*0.5)
		sp = np.sin(pitch*0.5)
		cy = np.cos(yaw*0.5)
		sy = np.sin(yaw*0.5)

		# this is the specific sequence for XYZ euler angles but it is different for other combinations
		w = cr*cp*cy - sr*sp*sy 
		x = sr*cp*cy + cr*sp*sy 
		y = cr*sp*cy - sr*cp*sy 
		z = cr*cp*sy + sr*sp*cy 
		return Quaternion(w=w, x=x, y=y, z=z)
	
	def update_ori(self, roll, pitch, yaw, slerp_amount=1.0):
		new_ori = self.__convert_from_euler_angles_to_quat(roll, pitch, yaw)
		interp_ori = Quaternion.slerp(self.ori, new_ori, slerp_amount)
		print(interp_ori)
		self.ori = interp_ori


	def plot(self):
		axis = plt.axes(projection='3d')
		self.plot_pose_axes(axis)
		axis.set_xlim(0, 6)
		axis.set_ylim(-3, 3)
		axis.set_zlim(0, 6)
		plt.show()

	def plot_pose_axes(self, axis: plt.Axes):
		x_axis = np.array([1, 0, 0])
		y_axis = np.array([0, 1, 0])
		z_axis = np.array([0, 0, 1])
		global_x_axis = self.ori.rotate(x_axis) + self.position
		global_y_axis = self.ori.rotate(y_axis) + self.position
		global_z_axis = self.ori.rotate(z_axis) + self.position
		# origin
		oX, oY, oZ = self.position
		iX, iY, iZ = global_x_axis
		jX, jY, jZ = global_y_axis
		kX, kY, kZ = global_z_axis

		axis.plot([oX, iX], [oY, iY], [oZ, iZ], 'r')
		axis.plot([oX, jX], [oY, jY], [oZ, jZ], 'g')
		axis.plot([oX, kX], [oY, kY], [oZ, kZ], 'b')

if __name__ == '__main__':
	object = SLERPTest([0, 0, 0], [0, 0, 0])
	object.plot()
	object.update_ori(0, 0, np.pi/2, 0.5)
	object.plot()
	print("Interpolated ori: ", object.ori)
