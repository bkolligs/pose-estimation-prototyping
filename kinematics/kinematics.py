import numpy as np
from rich.progress import track
import matplotlib.pyplot as plt
from four_wheel.four_wheel import FourWheel
from four_wheel.utils import drive_arc_convert
from progress.progress_bar import KinematicsProgress


class NoisyRover:
    def __init__(
        self,
        initial_state=[0, 0, 0, 0, 0, 0, 0],
        wheel_sigma=0.1,
        delta_t=0.01,
        max_speed=0.05,
    ) -> None:
        self.max_speed = max_speed
        self.delta_t = delta_t
        self.wheel_sigma = wheel_sigma
        self.kinematics = FourWheel(0.32195, 0.2222, 0.0745, 0.0955)
        # state is comprised of [body x, body y, body yaw, left front wheel location, right front wheel location, left back wheel location, right back wheel location]
        self.state = np.array(initial_state, dtype=float).reshape(-1, 1)
        self.states = []
        self.length = 0.1
        self.width = 0.2

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
        if not isinstance(wheel_vel, np.ndarray):
            wheel_vel = np.array(wheel_vel).reshape(-1, 1)
        wheel_vel = np.clip(wheel_vel, a_min=-self.max_speed, a_max=self.max_speed)
        self.runge_kutta_integration(wheel_vel)

    def plot_step(self, t=None):
        """
        Plots the intermediate state updates of the simulation
        """
        plt.clf()
        ax = plt.axes()
        # plot the location and orientation
        rectangle = plt.Rectangle(
            xy=(self.state[0], self.state[1]),
            width=self.width,
            height=self.length,
            angle=np.rad2deg(self.state[2]),
        )
        ax.add_patch(rectangle)
        if t is not None:
            ax.set_title(f"Simulation at time {t:.2f}")
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        plt.draw()
        plt.pause(0.01)


def plot_simulation(regular_rover: NoisyRover, noisy_rover: NoisyRover):
    """
    Plots the results of the simulation
    """
    plt.figure()
    noisy_states = np.array(noisy_rover.states)
    regular_states = np.array(regular_rover.states)
    plt.plot(
        noisy_states[:, 0],
        noisy_states[:, 1],
        "r",
        label=f"Noisy, $\sigma$={noisy_rover.wheel_sigma}",
    )
    plt.plot(regular_states[:, 0], regular_states[:, 1], "b", label="Ground Truth")
    plt.title("$x$ vs $y$ location, max speed = {0}".format(regular_rover.max_speed))
    plt.xlabel("$x$")
    plt.ylabel("$y$")
    plt.legend()
    plt.ylim(-2, 1)
    plt.xlim(-1, 2)
    plt.show()


def trajectory(t, max_time=10):
    """
    Time varying trajectory for the controls
    """
    time_splits = [0, 0.25 * max_time, 0.5 * max_time, 0.75 * max_time, max_time]

    if t > time_splits[0] and t <= time_splits[1]:
        return [0.02, 0.02, 0.02, 0.02]
    elif t > time_splits[1] and t <= time_splits[2]:
        return [0.05, 0.01, 0.05, 0.01]
    elif t > time_splits[2] and t <= time_splits[3]:
        return [0.01, 0.05, 0.01, 0.05]
    elif t > time_splits[3] and t <= time_splits[4]:
        return [0.05, 0.05, 0.05, 0.05]
    else:
        return [0.0, 0.0, 0.0, 0.0]


if __name__ == "__main__":
    # Initialize the fourwheel model with
    start, stop, h = 0, 300, 0.01
    times = np.arange(start, stop, h)
    rover = NoisyRover(wheel_sigma=0.0, delta_t=h)
    noisy_rover = NoisyRover(wheel_sigma=15.0, delta_t=h)

    with KinematicsProgress(len(times), np.zeros_like(rover.state)) as progress:
        for t in times:
            wheel_mean = trajectory(t, max_time=stop)
            control = noisy_rover.wheels(wheel_mean)
            noisy_rover.step(control)
            rover.step(wheel_mean)
            progress.step(rover.state, noisy_rover.state)

        print("Simulation Done")
        # print("Ground truth: \n", rover.state, "\nNoisy: \n", noisy_rover.state)
        plot_simulation(rover, noisy_rover)
