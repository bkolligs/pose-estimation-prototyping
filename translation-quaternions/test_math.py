import pytest
import numpy as np
from translation_from_imu import Rover

def test_quaternion_conversion():
	'''
	Ground truth obtained from https://quaternions.online/
	'''
	test_article = Rover([0, 0, 0], [0, 0, 0])
	ground_truth = [1.0, 0.0, 0.0, 0.0]

	assert test_article.ori.w == ground_truth[0]
	assert test_article.ori.x == ground_truth[1]
	assert test_article.ori.y == ground_truth[2]
	assert test_article.ori.z == ground_truth[3]


def test_quaternion_yaw():
	test_article = Rover([0, 0, np.pi/2], [0, 0, 0])
	ground_truth = [0.707, 0.0, 0.0, 0.707]

	assert np.allclose(test_article.ori.w, ground_truth[0], 1e-3)
	assert np.allclose(test_article.ori.x, ground_truth[1], 1e-3)
	assert np.allclose(test_article.ori.y, ground_truth[2], 1e-3)
	assert np.allclose(test_article.ori.z, ground_truth[3], 1e-3)

	test_article = Rover([0, 0, np.pi/4], [0, 0, 0])
	ground_truth = [0.924, 0.0, 0.0, 0.383]

	assert np.allclose(test_article.ori.w, ground_truth[0], 1e-3)
	assert np.allclose(test_article.ori.x, ground_truth[1], 1e-3)
	assert np.allclose(test_article.ori.y, ground_truth[2], 1e-3)
	assert np.allclose(test_article.ori.z, ground_truth[3], 1e-3)

	test_article = Rover([0, 0, -np.pi/4], [0, 0, 0])
	ground_truth = [0.924, 0.0, 0.0, -0.383]

	assert np.allclose(test_article.ori.w, ground_truth[0], 1e-3)
	assert np.allclose(test_article.ori.x, ground_truth[1], 1e-3)
	assert np.allclose(test_article.ori.y, ground_truth[2], 1e-3)
	assert np.allclose(test_article.ori.z, ground_truth[3], 1e-3)

def test_quaternion_roll():
	test_article = Rover([np.pi/4, 0, 0], [0, 0, 0])
	ground_truth = [0.924, 0.383, 0.0, 0.0]

	assert np.allclose(test_article.ori.w, ground_truth[0], 1e-3)
	assert np.allclose(test_article.ori.x, ground_truth[1], 1e-3)
	assert np.allclose(test_article.ori.y, ground_truth[2], 1e-3)
	assert np.allclose(test_article.ori.z, ground_truth[3], 1e-3)

def test_quaternion_pitch():
	test_article = Rover([0, np.pi/4, 0.0], [0, 0, 0])
	ground_truth = [0.924, 0.0, 0.383, 0]

	assert np.allclose(test_article.ori.w, ground_truth[0], 1e-3)
	assert np.allclose(test_article.ori.x, ground_truth[1], 1e-3)
	assert np.allclose(test_article.ori.y, ground_truth[2], 1e-3)
	assert np.allclose(test_article.ori.z, ground_truth[3], 1e-3)

def test_quaternion_full():
	test_article = Rover([0.175, 0.1, 0.1], [0, 0, 0])
	ground_truth = [0.993, 0.0896, 0.04536, 0.05408]

	assert np.allclose(test_article.ori.w, ground_truth[0], 1e-3)
	assert np.allclose(test_article.ori.x, ground_truth[1], 1e-3)
	assert np.allclose(test_article.ori.y, ground_truth[2], 1e-3)
	assert np.allclose(test_article.ori.z, ground_truth[3], 1e-3)