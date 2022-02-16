'''
This file tests out the current method pose estimation uses to estimate postiion and verify it works using velocities 
The approach uses the rover's orientation estimate as a quaternion and transforms the linear velocity of the rover into this rotated frame. 
Then adds the transformed position to the previous position
'''
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

class Rover:
	def __init__(self, initial_rotation: list, initial_position: list, timestep=0.05) -> None:
		self.starting_point = initial_position
		self.position = np.array(initial_position, dtype=float)
		# assuming initial rotation is given in roll, pitch, yaw order
		self.ori = self.__convert_from_euler_angles_to_quat(initial_rotation[0], initial_rotation[1], initial_rotation[2])
		self.timestep = timestep
		self.width, self.length, self.height = 0.1, 0.3, 0.2
		self.position_list = np.zeros(3)
		self.orientations = []
	
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

	def update_ori(self, new_ori: list):
		'''
		Updates the orientation
		'''
		roll, pitch, yaw = new_ori
		self.ori =  self.__convert_from_euler_angles_to_quat(roll, pitch, yaw)
		self.orientations.append(self.ori)
	
	def update_position(self, body_velocity):
		'''
		Uses the body velocity to update the position
		'''
		# local movement of the rover
		deltaP = np.array(
			[
				body_velocity * self.timestep,
				0,
				0
			]
		)

		updatedP = self.ori.rotate(deltaP)
		self.position += updatedP
		self.position_list = np.vstack((self.position_list, self.position))
	
	def plot_rover(self):
		plt.clf()
		axis = plt.axes(projection='3d')
		self.plot_pose_axes(axis)
		# plot the path traversed so far
		axis.plot(self.position_list[:, 0], self.position_list[:, 1], self.position_list[:, 2], 'k--')
		# plot the starting location
		axis.plot(self.starting_point[0], self.starting_point[1], self.starting_point[2], "go")
		axis.set_xlim(0, 6)
		axis.set_ylim(-3, 3)
		axis.set_zlim(0, 6)
		plt.draw()
		plt.pause(0.01)
	
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

def trajectory(time: float):
	if time > 0 and time < 3:
		return [0, 0, np.cos(time)*np.sin(time)]
	if time >= 3 and time < 9:
		return [0, 0.5, 0]
	if time >= 10 and time < 20:
		return [0, -0.5, np.cos(time)]
	if time >= 20 and time < 25:
		return [0, np.sin(t/3), 0]
	else: 
		return [0, 0, 0]


if __name__ == '__main__':
	rover = Rover([0, 0.0, 0.0], [0, 0, 0])
	times = np.arange(0, 30, 0.05)
	for t in times:
		ori = trajectory(t)
		rover.update_ori(ori)
		rover.update_position(0.2)
		rover.plot_rover()
	
	print("Simulation over. Final position: ", rover.position, "Final Ori: ", rover.ori)
	plt.show()