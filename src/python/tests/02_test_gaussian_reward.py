import pytest
import pickle
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from pylocogym.envs.rewards.dog import gaussian_reward as reward


pickle_path = '/pickled_files/test_data_dog_gaussian1678969681.2247353.pickle'
cfp = os.path.dirname(__file__)
fpath = ''.join([cfp, pickle_path])

test_data = pickle.load( open( fpath, "rb" ) )
r, info = reward.compute_reward(test_data['observation_raw'],
                                test_data['dt'],
                                test_data['num_joints'],
                                test_data['params'],
                                test_data['feet_status'],
                                test_data['all_torques'],
                                test_data['action_buffer'],
                                test_data['is_obs_fullstate'],
                                test_data['joint_angles_default'],
                                test_data['nominal_base_height'])


def test_forward_vel_reward():
    reward_term = 'forward_vel_reward'
    print("")
    print("Your", reward_term, ": ", info[reward_term])
    print("Test", reward_term, ": ", test_data['info'][reward_term])
    assert info[reward_term] == pytest.approx(test_data['info'][reward_term])


def test_torque_reward():
    reward_term = 'torque_reward'
    print("")
    print("Your", reward_term, ": ", info[reward_term])
    print("Test", reward_term, ": ", test_data['info'][reward_term])
    assert info[reward_term] == pytest.approx(test_data['info'][reward_term])


def test_smoothness1_reward():
    reward_term = 'smoothness1_reward'
    print("")
    print("Your", reward_term, ": ", info[reward_term])
    print("Test", reward_term, ": ", test_data['info'][reward_term])
    assert info[reward_term] == pytest.approx(test_data['info'][reward_term])


def test_smoothness2_reward():
    reward_term = 'smoothness2_reward'
    print("")
    print("Your", reward_term, ": ", info[reward_term])
    print("Test", reward_term, ": ", test_data['info'][reward_term])
    assert info[reward_term] == pytest.approx(test_data['info'][reward_term])


def test_smoothness_reward():
    reward_term = 'smoothness_reward'
    print("")
    print("Your", reward_term, ": ", info[reward_term])
    print("Test", reward_term, ": ", test_data['info'][reward_term])
    assert info[reward_term] == pytest.approx(test_data['info'][reward_term])


def test_reward():

    print("")
    print("Your reward: ", r)
    print("Test reward: ", test_data['reward'])
    assert r == pytest.approx(test_data['reward'])
