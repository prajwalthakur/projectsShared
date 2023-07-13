import numpy as np
import scipy.linalg
import sys

import os
sys.path.insert(1,os.getcwd())
from tools.tools import Quaternion2Euler , Euler2Quaternion
import pandas as pd

pd.set_option('display.width', 320)
pd.set_option('display.max_columns', 12)
np.set_printoptions(linewidth=320)

trim_input = np.array([[0.0001],
                       [-0.1083],
                       [-0.0001],
                       [0.33346]])

da_eq = trim_input.item(0)
de_eq = trim_input.item(1)
dr_eq = trim_input.item(2)
dt_eq = trim_input.item(3)

"""self._state = np.array([[MAV.pn0],  # (0)
                        [MAV.pe0],  # (1)
                        [MAV.pd0],  # (2)
                        [MAV.u0],  # (3)
                        [MAV.v0],  # (4)
                        [MAV.w0],  # (5)
                        [MAV.e0],  # (6)
                        [MAV.e1],  # (7)
                        [MAV.e2],  # (8)
                        [MAV.e3],  # (9)
                        [MAV.p0],  # (10)
                        [MAV.q0],  # (11)
                        [MAV.r0]])  # (12)"""
phi_eq = 0.0813
theta_eq = -0.0000
psi_eq = 0.0000
[e0,e1,e2,e3]= Euler2Quaternion(phi_eq, theta_eq, psi_eq )
trim_state = np.array([[0.00000000e+00],
                       [0.00000000e+00],
                       [-1.00000000e+02],
                       [24.9985],
                       [-0.0010],
                       [2.0364],
                       [e0],
                       [e1],
                       [e2],
                       [e3],
                       [0.00000000e+00],
                       [0.00000000e+00],
                       [0.00000000e+00]])

long_trim_state = np.array([
    [trim_state.item(3)],  # u
    [trim_state.item(5)],  # w
    [trim_state.item(11)],  # q
    [theta_eq],  # theta
    [-trim_state.item(2)]])  # h

### Lon with alpha
"""A_lon_cont = np.array(
    [[-0.57681981, 0.48178674 * 25., -1.21990873, -9.81 * np.cos(theta_eq), 0.],  # -9.78648014,   0.        ],
     [-0.56064823 / 25., -4.46355336, 24.37104644 / 25., -0.56472816 / 25., 0.],
     [0.19994698, -3.99297803 * 25., -5.2947383, 0., 0.],
     [0., 0., 1., 0., 0.],  # 0.99971072,   0.,           0.        ],
     [np.sin(theta_eq), -np.cos(theta_eq) * 25., 0.,
      trim_state.item(3) * np.cos(theta_eq) + trim_state.item(5) * np.sin(theta_eq), 0.]])

B_lon_cont = np.array([[-0.13839273, 47.015150132144704],  # 47.76297312],
                  [-2.58618378 / 25., 0.],
                  [-36.11238957, 0.],
                  [0., 0.],
                  [0., 0.]])"""

A_lon_cont = np.array(
    [[-0.5185, 0.4751, -2.0057, -9.778, 0.],  # -9.78648014,   0.        ],
     [-0.5964, -2.2830, 24.9153, -0.8067, 0.],
     [0.0456, -0.5526, -0.4988, 0., 0.],
     [0., 0., 1., 0., 0.],  # 0.99971072,   0.,           0.        ],
     [0.0822, -0.9966, 0, 25.0, 0.]])

B_lon_cont = np.array([[-0.4780, 40.6455],  # 47.76297312],
                       [5.7929, 0.],
                       [-18.2386, 0.],
                       [0., 0.],
                       [0., 0.]])
