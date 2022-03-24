import numpy as np
import matplotlib.pyplot as plt
from four_wheel.four_wheel import FourWheel
from four_wheel.utils import drive_arc_convert


class NoisyRover:
    def __init__(
        self, initial_state=[0, 0, 0, 0, 0, 0, 0], wheel_sigma=0.1, delta_t=0.01
    ) -> None:
        self.delta_t = delta_t
        self.wheel_sigma = wheel_sigma
        self.kinematics = FourWheel(0.32195, 0.2222, 0.0745, 0.0955)
        # state is comprised of [body x, body y, body yaw, left front wheel location, right front wheel location, left back wheel location, right back wheel location]
        self.state = np.array(initial_state, dtype=float).reshape(-1, 1)
        self.states = []
        self.length = 0.5
        self.width = 1

    def dynamics(self, state: np.ndarray, control: np.ndarray):
        """
        Simple rover dynamics for the kinematic bicycle model for a skid steer
        """
        kinematics = self.kinematics.navigation(control)
        v = kinematics[3]
        psi_dot = kinematics[2]
        # print(kinematics.shape, control.shape, state.shape)
        x_dot = np.array(
            [
                v * np.cos(state[2]),
                v * np.sin(state[2]),
                psi_dot,
                control[0],
                control[1],
                control[2],
                control[3],
            ]
        )
        return x_dot

    def runge_kutta_integration(self, control):
        """
        Runge kutta fourth order integration for accuracy
        """
        # cache the state
        self.states.append(self.state.reshape(-1).copy())
        # perform the integration
        k1 = self.dynamics(self.state, control)
        k2 = self.dynamics(self.state + 0.5 * self.delta_t * k1, control)
        k3 = self.dynamics(self.state + 0.5 * self.delta_t * k2, control)
        k4 = self.dynamics(self.state + self.delta_t * k3, control)
        self.state += (self.delta_t / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def wheels(self, mean: list) -> np.ndarray:
        mean = np.array(mean)
        noisy_encoders = mean.reshape(-1, 1) + np.random.normal(
            loc=0, scale=self.wheel_sigma, size=(len(mean), 1)
        )
        return noisy_encoders

    def step(self, wheel_vel: list):
        self.runge_kutta_integration(wheel_vel)
    

    def plot_step(self, t=None):
        plt.clf()
        ax = plt.axes()
        # plot the location and orientation
        rectangle = plt.Rectangle(xy=(self.state[0], self.state[1]), width=self.width, height=self.length, angle=np.rad2deg(self.state[2]))
        ax.add_patch(rectangle)
        if t is not None: ax.set_title(f"Simulation at time {t:.2f}")
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        plt.draw()
        plt.pause(0.01)
    
    def plot_simulation(self):
        states = np.array(rover.states)
        plt.figure()
        plt.plot(states[:, 0], states[:, 1])
        plt.title("X vs Y location")
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.ylim(-1, 1)
        plt.xlim(-1, 1)
        plt.show()

if __name__ == "__main__":
    # Initialize the fourwheel model with
    rover = NoisyRover(wheel_sigma=5.0)
    for t in np.arange(0, 1, rover.delta_t):
        control = rover.wheels([2.5, 2.5, 2.5, 2.5])
        rover.step(control)
        rover.plot_step(t)

    print("Simulation Done")
    print(rover.state)
    rover.plot_simulation()
