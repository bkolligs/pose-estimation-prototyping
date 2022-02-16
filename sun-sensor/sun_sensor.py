import numpy as np
import matplotlib.pyplot as plt
from robotics import Transform

class SunSensor:
	def __init__(self, initial_rotation: list, initial_position: list) -> None:
		self.pose = Transform(initial_position[0], initial_position[1], initial_position[2], initial_rotation[0], initial_rotation[1], initial_rotation[2])
		self.fov = 60

	def plot_pose(self, axis=None):
		"""
		Plot the current pose
		"""
		if axis is None:
			axis = plt.axes(projection='3d')

		self.pose.plot(detached=True, axis_obj=axis, scale_factor=0.5)
	
	def step(self, new_position: list, alpha:float, beta:float, verbose=False):
		"""
		new_position is x, y, z, r, p, y
		"""
		plt.clf()
		print(f"Alpha={alpha:.3f}, Beta={beta:.3f}", end='\r')
		axis = plt.axes(projection="3d")
		self.pose.update_transform(new_position[0], new_position[1], new_position[2], new_position[3], new_position[4], new_position[5])
		self.plot_pose(axis)
		# plot the ray between the sensor and ray frame
		ray, in_fov = self.plot_ray(axis, alpha, beta, 2)
		if verbose: print("Ray is: \n", ray)
		# recolor the ray if it's outside the sensor field of view
		if in_fov: ray_color = 'g'
		else: ray_color = 'r'
		axis.plot([self.pose.x, ray.x], [self.pose.y, ray.y], [self.pose.z, ray.z], ray_color, linewidth=4)
		# set view angle
		axis.view_init(elev=30, azim=30)
		axis.set_xlim(-2, 2)
		axis.set_ylim(-2, 2)
		axis.set_zlim(-2, 2)
		plt.draw()
		plt.pause(0.01)
	
	def calculate_ray(self, alpha, beta, scale=1):
		"""
		Calculate a unit vector sun ray given alpha and beta angles
		"""
		rz = 1/np.sqrt(np.tan(alpha)**2 + np.tan(beta)**2 + 1)
		rx = rz*np.tan(alpha)
		ry = rz*np.tan(beta)
		return np.array([rx, ry, rz])*scale
	
	def plot_ray(self, axis, alpha, beta, ray_scale=1):
		"""
		Plots the ray on the given axis and in the world coordinates
		"""
		ray_x, ray_y, ray_z = self.calculate_ray(alpha, beta, ray_scale)
		ray_transform = Transform(x=ray_x, y=ray_y, z=ray_z)

		ray_in_world = self.pose*ray_transform
		# keep track of being inside the field of view
		in_fov = False
		if np.rad2deg(abs(alpha)) < self.fov and np.rad2deg(abs(beta)) < self.fov:
			in_fov = True
			ray_in_world.plot(detached=True, axis_obj=axis, scale_factor=0.5)
		return ray_in_world, in_fov
	
	def incorrect(self):
		"""
		Animation of the incorrect way to think about this
		"""
		
		# rotate first 45 along the x axis, then the y axis
		alphas = np.arange(0, np.pi/4, 0.01)
		betas = np.arange(0, np.pi/4, 0.01)
		fig = plt.figure(figsize=(16, 8))
		axis = fig.add_subplot(1, 2, 1, projection='3d')
		history_axis = fig.add_subplot(1, 2, 2, projection='3d')
		axis.view_init(elev=0, azim=0)
		history_axis.view_init(elev=0, azim=0)
		naive_ori = Transform()
		naive_ori_dot = Transform()

		def plot_stuff(alpha, beta):
			"""
			Intermediate function for plotting all this stuff
			"""
			self.pose.plot(detached=True, axis_obj=axis, rgb_xyz=['k--']*3)

			naive_ori.update_transform(theta=-beta, phi=alpha)
			alpha_real, beta_real = np.arctan2(naive_ori.z_axis[0], naive_ori.z_axis[2]), np.arctan2(naive_ori.z_axis[1], naive_ori.z_axis[2])
			axis.set_title(f"Alpha={alpha_real:.3f}, Beta={beta_real:.3f}")
			naive_ori.plot( detached=True, axis_obj=axis)
			naive_ori_dot.plot(detached=True, axis_obj=axis, rgb_xyz=['r', 'g', 'bo'])
			
			hist_x, hist_y, hist_z = naive_ori.z_axis
			history_axis.set_title(f"Location of the z-axis point after rotation [0, 0, 1]:\n {np.round(naive_ori.z_axis, 3)}")
			history_axis.plot(hist_x, hist_y, hist_z, 'bo')

			# plot the ray between the sensor and ray frame
			history_axis.set_xlim(-1, 1)
			history_axis.set_ylim(-1, 1)
			history_axis.set_zlim(-1, 1)
			axis.set_xlim(-1, 1)
			axis.set_ylim(-1, 1)
			axis.set_zlim(-1, 1)
			plt.draw()
			plt.pause(0.05)

		for i in range(len(alphas)):
			axis.cla()
			plt.suptitle("45 deg Roll, followed by 45 deg pitch rotation: Varying Beta Only")
			beta = betas[i]
			plot_stuff(0, beta)
		
		for i in range(90):
			history_axis.view_init(elev=0, azim=i)
			axis.view_init(elev=0, azim=i)
			plt.draw()
			plt.pause(0.05)
		
		for i in range(len(betas)):
			axis.cla()
			plt.suptitle("45 deg Roll, followed by 45 deg pitch rotation: Varying Alpha Only")
			alpha = alphas[i]
			plot_stuff(alpha, beta)
		
		
		print("\nSimulation complete!")
		plt.show()



if __name__ == '__main__':
	ss = SunSensor([0, 0, 0], [0, 0, 0])
	for t in np.arange(np.deg2rad(-60), np.deg2rad(60), 0.01):
		ss.step([0, 0, 0, 0, 0.0, 0], alpha=-0.5*t, beta=t)
	print("\nSimulation Done")
	plt.show()

	# my first idea that is not correct due to the fact we're only constrained by one angle 
	ss.incorrect()

