import moonranger as m
import pytest
import numpy as np

def test_state():
	ekf = m.OrientationEKF()
	compare = np.zeros(7)
	test = ekf.getState()

