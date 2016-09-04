# -*- coding: utf-8 -*-
#       __QUADROTORDYNAMICS__
#       This file implements the dynamics of a
#       simple quadrotor micro aerial vehicle
#
#       Largely based on the work of https://github.com/nikhilkalige

import numpy as np
import pandas as pd
from scipy.integrate import odeint


def moments(ref_acc, angular_vel, inertia_matrix):
    """Compute the moments

    Parameters
    ----------
    ref_acc : numpy.array
        The desired angular acceleration that the system should achieve. This
        should be of form [dp/dt, dq/dt, dr/dt]
    angular_vel : numpy.array
        The current angular velocity of the system. This
        should be of form [p, q, r]

    Returns
    -------
    numpy.array
        The desired moments of the system
    """
    inverse_inertia = np.linalg.inv(inertia_matrix)
    p1 = np.dot(inverse_inertia, angular_vel)
    p2 = np.dot(inertia_matrix, angular_vel)
    cross = np.cross(p1, p2)
    value = ref_acc + cross
    return np.dot(inertia_matrix, value)


def angular_velocity_to_dt_eulerangles(angular_rotation_matrix, angular_vel):
    """Angular Velocity TO Euler Angles
    omega = angular velocity :- np.array([p, q, r])
    """
    rotation_matrix = np.linalg.inv(angular_rotation_matrix)
    return np.dot(rotation_matrix, angular_vel)


def angular_rotation_matrix(phi, theta, psi):
    """Rotation matix for Angular Velocity <-> Euler Angles Conversion
    Use inverse of the matrix to convert from angular velocity to euler rates
    """
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cthe = np.cos(theta)
    sthe = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    RotMatAngV = np.array([[1, 0, -sthe],
                           [0, cphi, cthe * sphi],
                           [0, -spsi, cthe * cphi]
                           ])
    return RotMatAngV


def rotation_matrix(phi, theta, psi):
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cthe = np.cos(theta)
    sthe = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    RotMat = np.array([[cthe * cpsi, sphi * sthe * cpsi - cphi * spsi, cphi * sthe * cpsi + sphi * spsi],
                       [cthe * spsi, sphi * sthe * spsi + cphi * cpsi, cphi * sthe * spsi - sphi * cpsi],
                       [-sthe, cthe * sphi, cthe * cphi]])
    return RotMat


def motor_thrust(config, moments, total_thrust):
    """Compute Motor Thrusts

    Parameters
    ----------
    moments : numpy.array
        [Mp, Mq, Mr] moments
    total_thrust : float
        The total thrust generated by all motors

    Returns
    -------
    numpy.array
        The thrust generated by each motor [T1, T2, T3, T4]
    """
    [Mp, Mq, Mr] = moments
    thrust = np.zeros(4)
    tmp1add = total_thrust + Mr / config['thrustToDrag']
    tmp1sub = total_thrust - Mr / config['thrustToDrag']

    tmp2p = 2 * Mp / config['length']
    tmp2q = 2 * Mq / config['length']

    thrust[0] = tmp1add - tmp2q
    thrust[1] = tmp1sub + tmp2p
    thrust[2] = tmp1add + tmp2q
    thrust[3] = tmp1sub - tmp2p

    return thrust / 4.0


class QuadrotorDynamics(object):
    def __init__(self, save_state=True, config=None, dt=0.005):
        """
        Quadrotor Dynamics Parameters
        ----------
        save_state: Boolean
            Decides whether the state of the system should be saved
            and returned at the end
        """
        self.config = {
            # Define constants
            'gravity': 9.81,  # Earth gravity [m s^-2]
            # Define Vehicle Parameters
            'mass': 1,  # Mass [kg]
            'length': 0.2,  # Arm length [m]
            # Cross-Intertia [Ixx, Iyy, Izz]
            'inertia': np.array([0.0053, 0.0053, 0.0086]),  # [kg m^2]
            'thrustToDrag': 0.018  # thrust to drag constant [m]
        }

        self.save_state = save_state
        self._dt = dt  # Simulation Step
        if config:
            self.config.update(config)

        state_columns = pd.MultiIndex.from_tuples(list(zip(
            ['position', 'position', 'position', 'velocity', 'velocity', 'velocity', 'orientation', 'orientation',
             'orientation', 'omega', 'omega', 'omega'],
            ['x', 'y', 'z', 'x', 'y', 'z', 'phi', 'theta', 'psi', 'phi_dot', 'theta_dot', 'psi_dot']
        )), names=['variable', 'axis'])

        self.df_state = pd.DataFrame(columns=state_columns)
        self.df_current_state = pd.DataFrame(np.zeros((1, 12)),
                                             columns=state_columns)
        self.t_start = 0
        self.current_state = np.zeros((12))

        state_dot_columns = pd.MultiIndex.from_tuples(list(zip(
            ['velocity', 'velocity', 'velocity', 'acceleration', 'acceleration', 'acceleration', 'omega', 'omega',
             'omega', 'omega_dot', 'omega_dot', 'omega_dot'],
            ['x', 'y', 'z', 'x', 'y', 'z', 'phi_dot', 'theta_dot', 'psi_dot', 'phi_dot_dot', 'theta_dot_dot',
             'psi_dot_dot'])),
            names=['variable', 'axis'])

        self.df_current_state_dot = pd.DataFrame(np.zeros((1, 12)),
                                                 index=[0],
                                                 columns=state_dot_columns)

    def motor_thrust(self, moments, total_thrust):
        """Compute Motor Thrusts

        Parameters
        ----------
        moments : numpy.array
            [Mp, Mq, Mr] moments
        total_thrust : float
            The total thrust generated by all motors

        Returns
        -------
        numpy.array
            The thrust generated by each motor [T1, T2, T3, T4]
        """
        [Mp, Mq, Mr] = moments
        thrust = np.zeros(4)
        tmp1add = total_thrust + Mr / self.config['thrustToDrag']
        tmp1sub = total_thrust - Mr / self.config['thrustToDrag']

        tmp2p = 2 * Mp / self.config['length']
        tmp2q = 2 * Mq / self.config['length']

        thrust[0] = tmp1add - tmp2q
        thrust[1] = tmp1sub + tmp2p
        thrust[2] = tmp1add + tmp2q
        thrust[3] = tmp1sub - tmp2p

        return thrust / 4.0

    def dt_eulerangles_to_angular_velocity(self, dtEuler, EulerAngles):
        """Euler angles derivatives TO angular velocities
        dtEuler = np.array([dphi/dt, dtheta/dt, dpsi/dt])
        """
        return np.dot(self.angular_rotation_matrix(EulerAngles), dtEuler)

    def acceleration(self, thrusts, df_state):
        """Compute the acceleration in inertial reference frame
        thrust = np.array([Motor1, .... Motor4])
        """
        force_z_body = np.sum(thrusts) / self.config['mass']
        rotation_mat = rotation_matrix(*df_state.orientation.values[0])
        force_body = np.array([0, 0, force_z_body])
        return np.dot(rotation_mat, force_body) - np.array([0, 0, self.config['gravity']])

    def angular_acceleration(self, df_state, thrust):
        """Compute the angular acceleration in body frame
        omega = angular velocity :- np.array([p, q, r])
        """
        [t1, t2, t3, t4] = thrust
        thrust_matrix = np.array([self.config['length'] * (t2 - t4),
                                  self.config['length'] * (t3 - t1),
                                  self.config['thrustToDrag'] * (t1 - t2 + t3 - t4)])

        inverse_inertia = np.linalg.inv(self.inertia_matrix)
        p1 = np.dot(inverse_inertia, thrust_matrix)
        p2 = np.dot(inverse_inertia, df_state.omega.values[0])
        p3 = np.dot(self.inertia_matrix, df_state.omega.values[0])
        cross = np.cross(p2, p3)
        return p1 - cross

    def moments(self, ref_acc, df_state):
        return moments(ref_acc, df_state.omega.values[0], self.inertia_matrix)

    def angular_rotation_matrix(self, df_state):
        return angular_rotation_matrix(*df_state.orientation.values[0])

    @property
    def inertia_matrix(self):
        return np.diag(self.config['inertia'])

    def update_state(self, piecewise_args):
        """Run the state update equations for self._dt seconds
        Update the current state of the system. It runs the model and updates
        its state to self._dt seconds.

        Parameters
        ----------
        piecewise_args : array
            It contains the parameters that are needed to run each section
            of the flight. It is an array of tuples.
            [(ct1, da1, t1), (ct2, da2, t2), ..., (ctn, dan, tn)]

            ct : float
                The collective thrust generated by all motors
            da : numpy.array
                The desired angular acceleration that the system should achieve.
                This should be of form [dp/dt, dq/dt, dr/dt]
            t: float
                Time for which this section should run and should be atleast twice
                self._dt
        """
        if self.save_state:
            overall_time = 0
            for section in piecewise_args:
                overall_time += section.t

        # Create variable to maintain state between integration steps
        self._omega = np.zeros(3)

        for section in piecewise_args:
            if section.t < (2 * self._dt):
                continue

            ts = np.arange(self.t_start, self.t_start + section.t, self._dt)

            output = odeint(self._integrator, self.current_state, ts,
                            args=(section.total_thrust, section.desired_angular_acc))

            if self.save_state:
                # Final state update
                self.df_state = self.df_state.append(pd.DataFrame(output, index=ts, columns=self.df_state.columns))

            self.t_start = ts[-1]
            self.current_state = output[-1]

        return self.df_state

    def _integrator(self, state, t, total_thrust, desired_angular_acc):
        """Callback function for scipy.integrate.odeint.
            At this point scipy is used to execute the forward integration

        Parameters
        ----------
        state : numpy.array
            System State: [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r]
        t : float
            Time

        Returns
        -------
        numpy.array
            Rates of the input state:
            [x_dot, y_dot, z_dot, xd_dot, yd_dot, zd_dot, phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot]
        """
        self.df_current_state.values[:] = state

        # omega = self.dt_eulerangles_to_angular_velocity(omega, euler)
        moments = self.moments(desired_angular_acc, self.df_current_state)
        thrusts = motor_thrust(self.config, moments, total_thrust)
        # Acceleration in inertial frame
        self.df_current_state_dot.loc[:, ('velocity')] = self.df_current_state.velocity.values[0]
        self.df_current_state_dot.loc[:, ('acceleration')] = self.acceleration(thrusts, self.df_current_state)
        self.df_current_state_dot.loc[:, ('omega')] = angular_velocity_to_dt_eulerangles(
            angular_rotation_matrix(*self.df_current_state.orientation.values[0]),
            self.df_current_state.omega.values[0])
        self.df_current_state_dot.loc[:, ('omega_dot')] = self.angular_acceleration(self.df_current_state, thrusts)
        return self.df_current_state_dot.values[0]
