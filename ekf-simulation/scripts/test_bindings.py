import moonranger as m
import pytest
import numpy as np

def test_state():
	"""
	Tests that the retrieval of state is correct upon initializing.
	"""
	ekf = m.OrientationEKF()
	compare = np.zeros(10)
	test = ekf.getState()
	assert np.allclose(compare, test)

