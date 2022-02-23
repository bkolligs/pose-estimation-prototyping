# Pose Estimation Prototyping

This repository exists to prototype the code written for the pose estimation system for the [ MoonRanger Lunar Rover ](https://labs.ri.cmu.edu/moonranger/).

## Modules
The main components tested here are as follows: 
1. [Sun Sensor](sun-sensor/sun_sensor.py): Tests for verifying the math that converts `alpha` and `beta` angles to a unit vector representation of a sun ray. 
2. [EKF Simulation](ekf-simulation/scipts): Python bindings for the current implementation of the orientation EKF so that we can test, verify and visualize the results of the math. 
3. [Translation Estimation](translation-quaternions): Tests that verify we can use wheel velocities to estimate 3D position for a certain timestep.

## EKF Simulation Setup
The main dependencies for this project include a robotics package I wrote that contains helpful visualization tools called [PyRobo](https://github.com/bkolligs/pyrobo), NumPy, [PyBind11](https://github.com/pybind/pybind11), and Matplotlib.

The easiest way to install all of these dependencies is to use a [Conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html) environment with the following command: 
```bash
conda env create -f environment.yml
```

Otherwise you will need to use `pip`:

```bash
pip install pybind11[global] 
pip install git+https://github.com/bkolligs/pyrobo.git@main
```

Then we can install all of the C++ code by performing the following commands in the top level directory: 

_pose-estimation-prototyping/_
```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

These commands build a shared library and install it in the `site-packages` directory corresponding to the current `python` executable, which should be in the`proto-pose` conda environment if you used conda. Otherwise it will correspond to your `python3` executable, so any files should be run with that version of python. For example, `python3 ekf_verify.py`. This creates a `python` package called `moonranger` that can be accessed like this:

```python
import moonranger as m

ekf = m.OrientationEKF()

# get the ekf's current state
print(ekf.get_state())
```
