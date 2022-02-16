# Planetary Robotics Eigen Fork
This is a fork of the original Eigen repository which is hosted on GitLab. The upstream remote is at [https://gitlab.com/libeigen/eigen.git](https://gitlab.com/libeigen/eigen.git). 
In order to update this from the fork perform the following steps: 
## Clone this repository 
Clone the repository and verify the remote. 
```bash
$ git clone https://github.com/PlanetaryRobotics/eigen.git
$ git remote -v
origin	https://github.com/PlanetaryRobotics/eigen.git (fetch)
origin	https://github.com/PlanetaryRobotics/eigen.git (push)
```
## Add the Eigen repository as an upstream remote
This is where we can access the Eigen repository stored on GitLab. After adding it as an upstream remote, verify it was added successfully. 
```bash
$ git remote add upstream https://gitlab.com/libeigen/eigen.git
$ git remote -v
origin	https://github.com/PlanetaryRobotics/eigen.git (fetch)
origin	https://github.com/PlanetaryRobotics/eigen.git (push)
upstream	https://gitlab.com/libeigen/eigen.git (fetch)
upstream	https://gitlab.com/libeigen/eigen.git (push)
```

## Merge changes
```bash
$ git pull upstream master
```
Merge the changes from the original repository. 



**Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.**

For more information go to http://eigen.tuxfamily.org/.

For ***pull request***, ***bug reports***, and ***feature requests***, go to https://gitlab.com/libeigen/eigen.
