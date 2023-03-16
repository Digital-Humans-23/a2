import numpy as np
from scipy.spatial.transform import Rotation


class ObservationData:
    def __init__(self, observation_raw, num_joints, is_obs_fullstate=True):
        self.observation = observation_raw
        self.num_obs = len(observation_raw)
        self.num_additional_obs = self.num_obs - (num_joints * 2 + 9) + 3 * is_obs_fullstate
        self.is_fullstate = is_obs_fullstate

        if is_obs_fullstate:
            """Fully observed observation convention:
                    observation = [ global position, Euler angles, joint angles,
                                    global velocity, angular velocity, joint velocity,
                                    additional observation elements]
                                    
                    position convention: x = left, y = up, z = forward
                    Euler angle convention: (yaw, pitch, roll)
            """

            # position:
            self.pos = observation_raw[0:3]  # x = left, y = up, z = forward
            self.x = observation_raw[0]
            self.y = observation_raw[1]
            self.z = observation_raw[2]

            # orientation:
            self.ori = observation_raw[3:6]  # Euler angles (yaw, pitch, roll)
            self.yaw = observation_raw[3]
            self.pitch = observation_raw[4]
            self.roll = observation_raw[5]

            # joint angles:
            self.joint_angles = observation_raw[6:6 + num_joints]

            # linear velocity:
            self.vel = observation_raw[6 + num_joints:9 + num_joints]  # in global coordinates
            rotation_to_local = Rotation.from_euler('yxz', -self.ori)
            self.local_vel = rotation_to_local.apply(self.vel)

            # angular velocity:
            self.ang_vel = observation_raw[9 + num_joints:12 + num_joints]  # in global coordinates

            # joint velocity:
            self.joint_vel = observation_raw[12 + num_joints:12 + 2 * num_joints]

        else:
            """Partially observed observation convention:
                   observation = [ base height (y), pitch, roll, joint angles,
                                   local velocity, angular velocity, joint velocity,
                                   additional observation elements]
           """

            # position:
            self.y = observation_raw[0]

            # orientation:
            self.pitch = observation_raw[1]
            self.roll = observation_raw[2]

            # joint angles:
            self.joint_angles = observation_raw[3:3 + num_joints]

            # linear velocity:
            self.local_vel = observation_raw[3 + num_joints: 6 + num_joints]  # in local coordinates

            # angular velocity:
            self.ang_vel = observation_raw[6 + num_joints:9 + num_joints]  # in local coordinates

            # joint velocity:
            self.joint_vel = observation_raw[9 + num_joints:9 + 2 * num_joints]

        def print_obs():
            print(observation_raw)


def calc_rms(signal_vector):
    """ Given a vector of signals, returns a vector of RMS (Root Mean Squared) of signals"""
    return np.sqrt(np.mean(signal_vector ** 2, axis=1))


def calc_max_abs(signal_vector):
    """ Given a vector of signals, returns a vector of the elements with max magnitude for each signal"""
    return np.absolute(signal_vector).max(1)


def calc_rms_torque(torques, num_joints):
    """ Given a vector of signals, returns a vector of RMS (Root Mean Squared) of signals"""
    rms_torque = np.zeros(num_joints)
    num_sim_steps_per_loop = int(len(torques) / num_joints)
    for i in range(num_sim_steps_per_loop):
        rms_torque += np.power(torques[i * num_joints:(i + 1) * num_joints], 2)
    rms_torque = np.sqrt(rms_torque / num_sim_steps_per_loop)
    return rms_torque


def calc_max_mag_torque(torques, num_joints):
    """ Given a vector of signals, returns a vector of the elements with max magnitude for each signal"""
    max_mag_torque = np.zeros(num_joints)
    num_sim_steps_per_loop = int(len(torques) / num_joints)
    for i in range(num_sim_steps_per_loop):
        for joint_idx in range(num_joints):
            if abs(torques[i * num_joints + joint_idx]) > abs(max_mag_torque[joint_idx]):
                max_mag_torque[joint_idx] = torques[i * num_joints + joint_idx]
    return max_mag_torque


def tail(vector, segment_length):
    return vector[len(vector) - segment_length:]


def calc_derivatives(buffer, dt, num_elements):
    """ Calculate first and second derivative given history buffer.
    buffer = [current, previous, past]
    """
    current = buffer[0:num_elements]
    past = buffer[num_elements:2 * num_elements]
    past_past = buffer[num_elements * 2: 3 * num_elements]
    x_dot = (current - past) / dt
    x_ddot = (current - 2 * past + past_past) / dt ** 2
    return x_dot, x_ddot
