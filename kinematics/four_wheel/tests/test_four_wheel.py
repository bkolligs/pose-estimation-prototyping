import pytest
import numpy as np
from four_wheel.four_wheel import FourWheel


@pytest.fixture
def four_wheel():
    return FourWheel(0.1, 0.2, 0.1, 0.07)


def test_jacobian(four_wheel: FourWheel):
    test = four_wheel.jacobian
    # fmt: off
    compare = np.array(
        [
            [0         , -0.1700  , -0.1000  , 1.0000  , 0       , 0       , -0.0700  , 0        , 0        , 0       ]        ,
            [0.1700    , 0        , 0.2000   , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
            [0.1000    , -0.2000  , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]        ,
            [0         , -0.1700  , 0.1000   , 1.0000  , 0       , 0       , 0        , -0.0700  , 0        , 0       ]        ,
            [0.1700    , 0        , 0.2000   , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
            [-0.1000   , -0.2000  , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]        ,
            [0         , -0.1700  , -0.1000  , 1.0000  , 0       , 0       , 0        , 0        , -0.0700  , 0       ]        ,
            [0.1700    , 0        , -0.2000  , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
            [0.1000    , 0.2000   , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]        ,
            [0         , -0.1700  , 0.1000   , 1.0000  , 0       , 0       , 0        , 0        , 0        , -0.0700 ]        ,
            [0.1700    , 0        , -0.2000  , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
            [-0.1000   , 0.2000   , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]
        ]
    )    
    assert np.allclose(test, compare)

def test_jacobian_size(four_wheel):
    '''
    Test jacobian size
    '''
    test = four_wheel.jacobian.shape
    compare = (12, 10)
    assert test == compare

# the following tests check sizes of various jacobians
def test_jacobian_body_size(four_wheel):
    test = four_wheel.body_jacobian.shape
    compare = (12, 6)
    assert test == compare

def test_jacobian_wheel_size(four_wheel):
    test = four_wheel.wheel_jacobian.shape
    compare = (12, 4)
    assert test == compare

def test_jacobian_joint_inv_size(four_wheel):
    test = four_wheel.inv_wheel_jacobian.shape
    compare = (4, 12)
    assert test == compare

def test_jacobian_body_inv_size(four_wheel):
    test = four_wheel.inv_body_jacobian.shape
    compare = (6, 12)
    assert test == compare

def test_wheel_jacobian_inv(four_wheel):
    '''
    Verify the static inverse matches the calculated inverse
    '''
    test = np.round(np.linalg.pinv(four_wheel.wheel_jacobian), 8)
    compare = four_wheel.inv_wheel_jacobian
    assert np.allclose(test, compare)

def test_body_jacobian_inv(four_wheel):
    '''
    Verify the static inverse matches the calculated inverse
    '''
    test = np.round(np.linalg.pinv(four_wheel.body_jacobian), 8)
    compare = four_wheel.inv_body_jacobian
    # four_wheel.assert_(np.testing.assert_allclose(test, compare))
    assert np.allclose(test, compare)

def test_actuation_kinematics_straight(four_wheel):
    '''
    Drive in a straigt line
    '''
    test = np.round(four_wheel.actuation([0, 0, 0, 1, 0, 0]), 4)
    compare = np.array([
        14.2857,
        14.2857,
        14.2857,
        14.2857,
    ]).reshape((-1, 1))
    assert np.allclose(test, compare)

def test_actuation_kinematics_arc(four_wheel):
    '''
    Drive in an arc
    '''
    test = np.round(four_wheel.actuation([0, 0, 0.1, 0.5, 0, 0]), 4)
    compare = np.array([
        7.0,
        7.2857,
        7.0,
        7.2857,
    ]).reshape((-1, 1))
    assert np.allclose(test, compare)

def test_actuation_kinematics_point_turn(four_wheel):
    '''
    Drive in a point turn
    '''
    test = np.round(four_wheel.actuation([0, 0, np.pi, 0.0, 0, 0]), 4)
    compare = np.array([
        -4.4880 ,
        4.4880  ,
        -4.4880 ,
        4.4880
    ]).reshape((-1, 1))
    assert np.allclose(test, compare)

def test_navigation_forward(four_wheel):
    '''
    Driving forward, determine body velocity
    '''
    test = np.round(four_wheel.navigation([0.1, 0.1, 0.1, 0.1]), 4)
    compare = np.array([
        0, 
        0, 
        0, 
        0.007,
        0, 
        0
    ]).reshape((-1, 1))
    assert np.allclose(test, compare)

def test_navigation_arc(four_wheel):
    '''
    Driving forward, determine body velocity
    '''
    test = np.round(four_wheel.navigation([0.1, 0.3, 0.1, 0.3]), 4)
    compare = np.array([
        0, 
        0, 
        0.014,
        0.014,
        0, 
        0
    ]).reshape((-1, 1))
    assert np.allclose(test, compare)

def test_navigation_disparate_wheels(four_wheel):
    '''
    Say the wheels are moving at different velocities, and a couple are moving slower than desired
    '''
    test = np.round(four_wheel.navigation([0.15, 0.33, 0.08, 0.25]), 4)
    compare = np.array([
        0, 
        0, 
        0.0122,
        0.0142,
        0, 
        0
    ]).reshape((-1, 1))
    assert np.allclose(test, compare)