"""
Computing reward for Vanilla setup, constant target speed, quadratic kernels
"""
import numpy as np
from numpy.linalg import norm
from pylocogym.envs.rewards.utils.utils import *

import pickle
import time


def compute_reward(observation_raw, dt, num_joints, params, feet_status, all_torques, action_buffer, is_obs_fullstate,
                   joint_angles_default, nominal_base_height):
    """
    Compute the reward based on observation (Vanilla Environment).

    :param observation_raw: current observation
    :param dt: control time step size
    :param num_joints: number of joints
    :param params: reward params read from the config file
    :param feet_status: pos, vel and swing status of robot feet
    :param all_torques: torque records during the last control timestep
    :param action_buffer: history of previous actions
    :param is_obs_fullstate: flag to choose full state obs or not.
    :param joint_angles_default: default joint angles
    :return: total reward, reward information (different terms can be passed here to be plotted in the graphs)
    """

    observation = ObservationData(observation_raw, num_joints, is_obs_fullstate)
    action_dot, action_ddot = calc_derivatives(action_buffer, dt, num_joints)
    cmd_fwd_vel = params.get("fwd_vel_cmd", 1.0)  # Note: this is only correct when we have constant target speed
    torque = tail(all_torques, num_joints)

    # =============
    # define reward terms here:
    # =============
    # TODO: Implement the rewards here.
    # Hints:
    # - Use function params.get("weight_velocity", 0) to get the value of parameters set in the .conf file.
    forward_vel_reward = params.get("weight_velocity", 0) * 0

    torque_reward =  0

    smoothness1_reward = 0

    smoothness2_reward = 0

    # =============
    # sum up rewards
    # =============
    smoothness_reward = smoothness1_reward + smoothness2_reward
    reward = torque_reward + forward_vel_reward + smoothness_reward

    info = {
        "forward_vel_reward": forward_vel_reward,
        "torque_reward": torque_reward,
        "smoothness1_reward": smoothness1_reward,
        "smoothness2_reward": smoothness2_reward,
        "smoothness_reward": smoothness_reward,
    }

    return reward, info


def punishment(current_step, max_episode_steps, params):  # punishment for early termination
    penalty = params['weight_early_penalty'] * (max_episode_steps - current_step)
    return penalty


