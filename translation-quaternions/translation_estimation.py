'''
This file is verifying the translation estimation of the pose estimator, which takes in a locally referenced body velocity and integrates it additively
'''
import numpy as np
import matplotlib.pyplot as plt

class Rover:
    def __init__(self, initial_state: np.ndarray, timestep: float):
        self.state = np.array(initial_state, dtype=float)
        self.delta_t = timestep
        self.length = 0.5
        self.width = 1

    def dynamics(self, state, linear, angular):
        x_dot = np.array(
            [ 
                linear*np.cos(state[2]),
                linear*np.sin(state[2]),
                angular
            ]
        )
        return x_dot
    
    def step(self, linear, angular, method="rk4"):
        if method == "rk4":
            self.runge_kutta_integration(linear, angular)
            return 

        # euler integration
        self.state += self.dynamics(self.state, linear, angular) * self.delta_t
    
    def runge_kutta_integration(self, linear, angular):
        '''
        Runge kutta fourth order integration for accuracy
        '''
        k1 = self.dynamics(self.state, linear, angular)
        k2 = self.dynamics(self.state + 0.5*self.delta_t*k1, linear, angular)
        k3 = self.dynamics(self.state + 0.5*self.delta_t*k2, linear, angular)
        k4 = self.dynamics(self.state + self.delta_t*k3, linear, angular)
        self.state += (self.delta_t/6.0) *(k1 + 2*k2 + 2*k3 + k4)
    
    def plot_simulation(self, t=None):
        plt.clf()
        ax = plt.axes()
        # plot the location and orientation
        rectangle = plt.Rectangle(xy=(self.state[0], self.state[1]), width=self.width, height=self.length, angle=np.rad2deg(self.state[2]))
        ax.add_patch(rectangle)
        if t is not None: ax.set_title(f"Simulation at time {t:.2f}")
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)
        plt.draw()
        plt.pause(0.01)

def trajectory(time: float):
    if time > 0 and time < 2:
        return [0.2, 0]
    
    if time >= 2 and time < 4:
        return [0.2, 0.2]
    if time >= 4 and time < 8:
        return [0.2, 0]
    if time >= 8 and time < 10:
        return [0.1, 0.1]
    else: 
        return [0, 0]


if __name__ == '__main__':
    start, stop, dt = 0, 10, 0.1
    times = np.arange(start, stop, dt)
    print(len(times))
    rover = Rover([0, 0, 0], dt)
    for t in times:
        v, omega = trajectory(t)
        rover.step(v, omega)
        rover.plot_simulation(t=t)

    print(rover.state)
    print("Simulation Done")
    plt.show()
