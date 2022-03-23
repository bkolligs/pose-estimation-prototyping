# ============================================================================
#
#       Filename:  four_wheel.py
#
#    Description:  Contains the fourwheel kinematics model, as described in the design document. Contains parameters that can be tuned to estimate slip including the contact constraint and wheel radius.
#
#        Version:  1.0
#        Created:  09/11/2021
#       Revision:  none
#
#         Author:  Ben Kolligs
#          Email:  bkolligs@andrew.cmu.edu
#   Organization:  Planetary Robotics Lab
#
# ============================================================================
import numpy as np


class FourWheel:
    def __init__(self, w, l, h, r, vx=0, vy=0) -> None:
        """
        This initializes the four wheeled kinematic model using wheel jacobians
        """
        # width of the vehicle from the center frame (total vehicle is 2w)
        self.width = w
        # length of vehicle from center frame (total vehicle is 2l)
        self.length = l
        self.height = h
        self.wheel_radius = r
        # velocity constraints for the contact points, this is a knob to tune slip estimate
        self.v_constraints = np.zeros((12, 1))
        self.v_constraints[[0, 3, 6, 9]] = vx
        self.v_constraints[[1, 4, 7, 10]] = vy
        self.jacobian = np.array(
            [
                [0, -h - r, -w, 1, 0, 0, -r, 0, 0, 0],
                [h + r, 0, l, 0, 1, 0, 0, 0, 0, 0],
                [w, -l, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, -h - r, w, 1, 0, 0, 0, -r, 0, 0],
                [h + r, 0, l, 0, 1, 0, 0, 0, 0, 0],
                [-w, -l, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, -h - r, -w, 1, 0, 0, 0, 0, -r, 0],
                [h + r, 0, -l, 0, 1, 0, 0, 0, 0, 0],
                [w, l, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, -h - r, w, 1, 0, 0, 0, 0, 0, -r],
                [h + r, 0, -l, 0, 1, 0, 0, 0, 0, 0],
                [-w, l, 0, 0, 0, 1, 0, 0, 0, 0],
            ]
        )
        # slice up the jacobian
        self.body_jacobian = self.jacobian[:, :6]
        self.wheel_jacobian = self.jacobian[:, 6:]
        self.inv_wheel_jacobian = np.array(
            [
                [-1 / r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, -1 / r, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, -1 / r, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / r, 0, 0],
            ]
        )

        # predefine some things for ease
        a = 1 / (w)
        b = 1 / (l)
        c = w / ((l**2 + w**2))
        d = l / ((l**2 + w**2))
        e = (r * h**2 + h * r**2) / (h * r * l)
        f = (r * h**2 + h * r**2) / (h * r * w)
        self.inv_body_jacobian = (
            1
            / 4
            * np.array(
                [
                    [0, 0, a, 0, 0, -a, 0, 0, a, 0, 0, -a],
                    [0, 0, -b, 0, 0, -b, 0, 0, b, 0, 0, b],
                    [-c, d, 0, c, d, 0, -c, -d, 0, c, -d, 0],
                    [1, 0, -e, 1, 0, -e, 1, 0, e, 1, 0, e],
                    [0, 1, -f, 0, 1, f, 0, 1, -f, 0, 1, f],
                    [0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1],
                ]
            )
        )

    def actuation(self, body_velocity):
        """
        Perform the actuation kinematics: body -> joint.
        Output is in radians per second
        """
        if not isinstance(body_velocity, np.ndarray):
            body_velocity = np.array(body_velocity)
            body_velocity = body_velocity.reshape((-1, 1))
        assert body_velocity.shape == (6, 1)

        return self.inv_wheel_jacobian @ (
            self.v_constraints - self.body_jacobian @ body_velocity
        )

    def navigation(self, wheel_velocity):
        """
        Perform the navigation kinematics: joint -> body.
        Output is in meters per second
        """
        if not isinstance(wheel_velocity, np.ndarray):
            wheel_velocity = np.array(wheel_velocity)
            wheel_velocity = wheel_velocity.reshape((-1, 1))
        assert wheel_velocity.shape == (4, 1)

        return self.inv_body_jacobian @ (
            self.v_constraints - self.wheel_jacobian @ wheel_velocity
        )
