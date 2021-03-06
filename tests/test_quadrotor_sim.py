# -*- coding: utf-8 -*-
#       __QUADROTORSIMSPARAMS__
#       This file implements the sim parameters
#       simple quadrotor simulation tool
#
#       Largely based on the work of https://github.com/nikhilkalige

from collections import namedtuple
import numpy as np
import pytest

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
        p1 = 0.2
        acc_start = self.get_acceleration(p0, p3)['start']
        return [p0, p1, p3]

    def get_sections(self, parameters):
        [p0, p1, p3] = parameters

        ap = self.get_acceleration(p0, p3)

        T2 = (self.Cpmax - p1 * ap['acc']) / ap['start']

        aq = 0
        ar = 0

        return (
            Section(
                total_thrust=self.mass * p0,
                desired_angular_acc=[ap['acc'], aq, ar],
                t=p1),
            Section(
                total_thrust=self.mass * self.Bup - 2 * abs(ap['start']) * self.Ixx / self.length,
                desired_angular_acc=[ap['start'], aq, ar],
                t=T2),
        )
        return sections


gen = SimulationParams()
quadrotor = QuadrotorDynamics()
params = gen.get_initial_parameters()
sections = gen.get_sections(params)
state = quadrotor.update_state(sections)


def test_state_position_x():
    np.testing.assert_allclose(state.position.x.values, np.zeros(83))


def test_state_position_y():
    np.testing.assert_allclose(state.position.y.values, np.array(
        [0.00000000e+00, 1.03717735e-08, 1.64897854e-07, 8.34271060e-07, 2.63630637e-06, 6.43594093e-06,
         1.33452279e-05, 2.47233189e-05, 4.21764376e-05, 6.75578372e-05, 1.02967720e-04, 1.50753162e-04, 2.13507933e-04,
         2.94072359e-04, 3.95532984e-04, 5.21222470e-04, 6.74718958e-04, 8.59845657e-04, 1.08067030e-03, 1.34150433e-03,
         1.64690217e-03, 2.00166005e-03, 2.41081497e-03, 2.87964329e-03, 3.41365895e-03, 4.01861203e-03, 4.70048624e-03,
         5.46549694e-03, 6.32008834e-03, 7.27093048e-03, 8.32491615e-03, 9.48915680e-03, 1.07709790e-02, 1.21779204e-02,
         1.37177233e-02, 1.53983303e-02, 1.72278782e-02, 1.92146918e-02, 2.13672771e-02, 2.36943131e-02, 2.36943131e-02,
         2.61725927e-02, 2.87763119e-02, 3.15094464e-02, 3.43747401e-02, 3.73737209e-02, 4.05067090e-02, 4.37728186e-02,
         4.71699532e-02, 5.06947948e-02, 5.43427851e-02, 5.81081013e-02, 6.19836302e-02, 6.59609392e-02, 7.00302454e-02,
         7.41803867e-02, 7.83987990e-02, 8.26714988e-02, 8.69830778e-02, 9.13167120e-02, 9.56541893e-02, 9.99759610e-02,
         1.04261220e-01, 1.08488014e-01, 1.12633392e-01, 1.16673599e-01, 1.20584306e-01, 1.24340901e-01, 1.27918824e-01,
         1.31293958e-01, 1.34443068e-01, 1.37344296e-01, 1.39977697e-01, 1.42325811e-01, 1.44374266e-01, 1.46112388e-01,
         1.47533807e-01, 1.48637040e-01, 1.49426020e-01, 1.49910542e-01, 1.50106611e-01, 1.50036636e-01, 1.49729464e-01,
         ]))


def test_state_position_z():
    np.testing.assert_allclose(state.position.z.values, np.array(
        [0.00000000e+00, 1.20150003e-04, 4.80599966e-04, 1.08134957e-03, 1.92239775e-03, 3.00374156e-03,
         4.32537521e-03, 5.88728787e-03, 7.68946202e-03, 9.73187077e-03, 1.20144751e-02, 1.45372206e-02, 1.73000340e-02,
         2.03028192e-02, 2.35454525e-02, 2.70277790e-02, 3.07496069e-02, 3.47107008e-02, 3.89107773e-02, 4.33494993e-02,
         4.80264680e-02, 5.29412170e-02, 5.80932046e-02, 6.34818063e-02, 6.91063066e-02, 7.49658910e-02, 8.10596371e-02,
         8.73865057e-02, 9.39453315e-02, 1.00734813e-01, 1.07753505e-01, 1.14999803e-01, 1.22471938e-01, 1.30167965e-01,
         1.38085749e-01, 1.46222957e-01, 1.54577043e-01, 1.63145240e-01, 1.71924548e-01, 1.80911719e-01, 1.80911719e-01,
         1.90026252e-01, 1.99188558e-01, 2.08396905e-01, 2.17650063e-01, 2.26947330e-01, 2.36288559e-01, 2.45674157e-01,
         2.55105076e-01, 2.64582789e-01, 2.74109245e-01, 2.83686809e-01, 2.93318193e-01, 3.03006366e-01, 3.12754441e-01,
         3.22565556e-01, 3.32442724e-01, 3.42388676e-01, 3.52405679e-01, 3.62495337e-01, 3.72658379e-01, 3.82894426e-01,
         3.93201754e-01, 4.03577034e-01, 4.14015083e-01, 4.24508601e-01, 4.35047921e-01, 4.45620772e-01, 4.56212063e-01,
         4.66803705e-01, 4.77374468e-01, 4.87899905e-01, 4.98352336e-01, 5.08700923e-01, 5.18911833e-01, 5.28948512e-01,
         5.38772079e-01, 5.48341840e-01, 5.57615940e-01, 5.66552143e-01, 5.75108737e-01, 5.83245558e-01, 5.90925105e-01,
         ]))


def test_orientation_position_phi():
    np.testing.assert_allclose(state.orientation.phi.values, np.array(
        [+0.00000000e+00, -2.54481140e-04, -1.01792454e-03, -2.29033020e-03, -4.07169812e-03,
         -6.36202831e-03, -9.16132076e-03, -1.24695755e-02, -1.62867925e-02, -2.06129717e-02, -2.54481132e-02,
         -3.07922170e-02, -3.66452830e-02, -4.30073113e-02, -4.98783019e-02, -5.72582547e-02, -6.51471698e-02,
         -7.35450472e-02, -8.24518868e-02, -9.18676887e-02, -1.01792453e-01, -1.12226179e-01, -1.23168868e-01,
         -1.34620519e-01, -1.46581132e-01, -1.59050708e-01, -1.72029245e-01, -1.85516745e-01, -1.99513208e-01,
         -2.14018632e-01, -2.29033019e-01, -2.44556368e-01, -2.60588679e-01, -2.77129953e-01, -2.94180189e-01,
         -3.11739387e-01, -3.29807547e-01, -3.48384670e-01, -3.67470755e-01, -3.87065802e-01, -3.87065802e-01,
         -4.04832781e-01, -4.18434668e-01, -4.27871460e-01, -4.33143158e-01, -4.34249762e-01, -4.31191271e-01,
         -4.23967687e-01, -4.12579007e-01, -3.97025234e-01, -3.77306366e-01, -3.53422404e-01, -3.25373347e-01,
         -2.93159196e-01, -2.56779951e-01, -2.16235611e-01, -1.71526177e-01, -1.22651649e-01, -6.96120261e-02,
         -1.24073092e-02, +4.89625022e-02, +1.14497408e-01, +1.84197408e-01, +2.58062502e-01, +3.36092691e-01,
         +4.18287974e-01, +5.04648351e-01, +5.95173823e-01, +6.89864389e-01, +7.88720049e-01, +8.91740804e-01,
         +9.98926653e-01, +1.11027760e+00, +1.22579363e+00, +1.34547477e+00, +1.46932099e+00, +1.59733231e+00,
         +1.72950873e+00, +1.86585024e+00, +2.00635684e+00, +2.15102854e+00, +2.29986533e+00, +2.45286722e+00]))


def test_orientation_position_theta():
    np.testing.assert_allclose(state.orientation.theta.values, np.zeros(83))


def test_orientation_position_psi():
    np.testing.assert_allclose(state.orientation.psi.values, np.zeros(83))


def test_state_index():
    np.testing.assert_allclose(state.orientation.index.values, np.array(
        [0., 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065, 0.07, 0.075, 0.08,
         0.085, 0.09, 0.095, 0.1, 0.105, 0.11, 0.115, 0.12, 0.125, 0.13, 0.135, 0.14, 0.145, 0.15, 0.155, 0.16, 0.165,
         0.17, 0.175, 0.18, 0.185, 0.19, 0.195, 0.195, 0.2, 0.205, 0.21, 0.215, 0.22, 0.225, 0.23, 0.235, 0.24, 0.245,
         0.25, 0.255, 0.26, 0.265, 0.27, 0.275, 0.28, 0.285, 0.29, 0.295, 0.3, 0.305, 0.31, 0.315, 0.32, 0.325, 0.33,
         0.335, 0.34, 0.345, 0.35, 0.355, 0.36, 0.365, 0.37, 0.375, 0.38, 0.385, 0.39, 0.395, 0.4, 0.405]))

@pytest.mark.xfail
@pytest.mark.parametrize("state_value, state_history",[
    (state.orientation.index.values,quadrotor.df_state_history.index.values),
    (state.orientation.psi.values,quadrotor.df_state_history.orientation.psi.values),
    (state.orientation.phi.values,quadrotor.df_state_history.orientation.phi.values),
    (state.orientation.theta.values,quadrotor.df_state_history.orientation.theta.values),
    (state.position.x.values,quadrotor.df_state_history.position.x.values),
    (state.position.y.values,quadrotor.df_state_history.position.y.values),
    (state.position.z.values,quadrotor.df_state_history.position.z.values),
])
def test_df_state_history_against_state(state_value, state_history):
    np.testing.assert_allclose(state_value, state_history)
    
    