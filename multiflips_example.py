# -*- coding: utf-8 -*-
#       __QUADROTORSIMSPARAMS__
#       This file implements the sim parameters
#       simple quadrotor simulation tool
#
#       Largely based on the work of https://github.com/nikhilkalige

from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np

from quadrotor_simulator.quadrotor_dynamics import QuadrotorDynamics

Section = namedtuple('Section', ['total_thrust', 'desired_angular_acc', 't'])

TURNS = 3


class SimulationParams(object):
    def __init__(self):
        self.mass = 1
        self.Ixx = 0.0053
        self.length = 0.2
        self.Bup = 21.58
        self.Bdown = 3.92
        self.Cpmax = np.pi * 1800 / 180
        self.Cn = TURNS
        self.gravity = 9.81

    def get_acceleration(self, p0, p3):
        ap = {
            'acc': (-self.mass * self.length * (self.Bup - p0) / (4 * self.Ixx)),
            'start': (self.mass * self.length * (self.Bup - self.Bdown) / (4 * self.Ixx)),
            'coast': 0,
            'stop': (-self.mass * self.length * (self.Bup - self.Bdown) / (4 * self.Ixx)),
            'recover': (self.mass * self.length * (self.Bup - p3) / (4 * self.Ixx)),
        }
        return ap

    def get_initial_parameters(self):
        p0 = p3 = 0.9 * self.Bup
        p1 = p4 = 0.2
        acc_start = self.get_acceleration(p0, p3)['start']
        p2 = (2 * np.pi * self.Cn / self.Cpmax) - (self.Cpmax / acc_start)
        return [p0, p1, p2, p3, p4]

    def get_sections(self, parameters):
        sections = np.zeros(5, dtype='object')
        [p0, p1, p2, p3, p4] = parameters

        ap = self.get_acceleration(p0, p3)

        T2 = (self.Cpmax - p1 * ap['acc']) / ap['start']
        T4 = -(self.Cpmax + p4 * ap['recover']) / ap['stop']

        aq = 0
        ar = 0

        sections[0] = Section(
            total_thrust=self.mass * p0,
            desired_angular_acc=[ap['acc'], aq, ar],
            t=p1)

        sections[1] = Section(
            total_thrust=self.mass * self.Bup - 2 * abs(ap['start']) * self.Ixx / self.length,
            desired_angular_acc=[ap['start'], aq, ar],
            t=T2)

        sections[2] = Section(
            total_thrust=self.mass * self.Bdown,
            desired_angular_acc=[ap['coast'], aq, ar],
            t=p2)

        sections[3] = Section(
            total_thrust=self.mass * self.Bup - 2 * abs(ap['stop']) * self.Ixx / self.length,
            desired_angular_acc=[ap['stop'], aq, ar],
            t=T4)

        sections[4] = Section(
            total_thrust=self.mass * p3,
            desired_angular_acc=[ap['recover'], aq, ar],
            t=p4)
        return sections


if __name__ == "__main__":
    gen = SimulationParams()
    quadrotor = QuadrotorDynamics()
    params = gen.get_initial_parameters()
    sections = gen.get_sections(params)
    state = quadrotor.update_state(sections)
    for variable in ["position", "velocity", "orientation", "omega"]:
        fig = state[variable].plot(title=variable)
        plt.gcf().canvas.set_window_title(variable)
    plt.show()