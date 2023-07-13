"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

"""
import sys
import os

import os
sys.path.insert(1,os.getcwd())
import numpy as np
import pdb
# load message types
from message_types.msg_state import msg_state

import parameters.aerosonde_parameters as MAV

sys.path.append('../tools')
from tools.angleConversions import Quaternion2Euler, Quaternion2Rotation
import math as mt
from scipy import interpolate


class mav_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file _state is the 13x1 internal state of the aircraft that is being
        # propagated: _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r] We will also need a variety of other
        # elements that are functions of the _state and the wind. self.true_state is a 19x1 vector that is estimated
        # and used by the autopilot to control the aircraft: true_state = [pn, pe, h, Va, alpha, beta, phi, theta,
        # chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.pn0],  # (0)
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
                                [MAV.r0]])  # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self._update_velocity_data(np.zeros((6, 1)))
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.Va0
        self._Vg = MAV.Va0
        self._alpha = 0
        self._beta = 0
        self._chi = 0  # could initialize better
        self._gamma = 0  # could initialize better
        # initialize true_state message
        self.msg_true_state = msg_state()
        self._update_msg_true_state()  # initial trim state should be updated before starting of any loop.

    ###################################
    # public functions
    def update_state(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step / 2. * k1, forces_moments)
        k3 = self._derivatives(self._state + time_step / 2. * k2, forces_moments)
        k4 = self._derivatives(self._state + time_step * k3, forces_moments)
        self._state += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0 ** 2 + e1 ** 2 + e2 ** 2 + e3 ** 2)
        self._state[6][0] = self._state.item(6) / normE
        self._state[7][0] = self._state.item(7) / normE
        self._state[8][0] = self._state.item(8) / normE
        self._state[9][0] = self._state.item(9) / normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_msg_true_state()

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics x_dot = f(x, u), returns f(x, u)
        """
        # extract the states
        # A_lon = A_e[[[3], [5], [10], [7], [2]], [3, 5, 10, 7, 2]]
        # u = state.item(3)
        # w = state.item(5)
        # p = state.item(10)
        # e1 = state.item(7)
        # pd = state.item(2)
        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pn_dot = (e1 ** 2 + e0 ** 2 - e2 ** 2 - e3 ** 2) * u + \
                 (2 * (e1 * e2 - e3 * e0)) * v + \
                 (2 * (e1 * e3 + e2 * e0)) * w
        pe_dot = (2 * (e1 * e2 + e3 * e0)) * u + \
                 (e2 ** 2 + e0 ** 2 - e1 ** 2 - e3 ** 2) * v + \
                 (2 * (e2 * e3 - e1 * e0)) * w
        pd_dot = (2 * (e1 * e3 - e2 * e0)) * u + \
                 (2 * (e2 * e3 + e1 * e0)) * v + \
                 (e3 ** 2 + e0 ** 2 - e1 ** 2 - e2 ** 2) * w
        # E_Matrix = np.array([[(e1**2 + e0**2 - e2**2 - e3**2), (2*(e1*e2-e3*e0)), (2*(e1*e3 + e2*e0))],
        #                      [(2*(e1*e2 + e3*e0)), (e2**2 + e0**2 - e1**2 - e3**2), (2*(e2*e3 - e1*e0))],
        #                      [(2*(e1*e3 - e2*e0)), (2*(e2*e3 + e1*e0)), (e3**2 + e0**2 - e1**2 - e2**2)]])
        # u_v_w = np.array([[u],[v],[w]])
        # ped_dot = E_Matrix @ u_v_w
        # pn_dot = ped_dot[0]
        # pe_dot = ped_dot[1]
        # pd_dot = ped_dot[2]

        # position dynamics
        u_dot = r * v - q * w + fx / MAV.mass
        v_dot = p * w - r * u + fy / MAV.mass
        w_dot = q * u - p * v + fz / MAV.mass

        # rotational kinematics
        e0_dot = (-p * e1 - q * e2 - r * e3) / 2
        e1_dot = (p * e0 + r * e2 - q * e3) / 2
        e2_dot = (q * e0 - r * e1 + p * e3) / 2
        e3_dot = (r * e0 + q * e1 - p * e2) / 2

        # # normalization similink thing
        # lamda = 100
        # norm_E = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        # rotate_E = np.array([[(lamda*(1-norm_E**2)), -p, -q, -r], \
        #                     [p, (lamda*(1-norm_E**2)), r, -q], \
        #                     [q, -r , (lamda*(1-norm_E**2)),p], \
        #                     [r, q, -p, (lamda*(1-norm_E**2))]])
        # e_dot = .5 * rotate_E @ np.array([[e0],[e1],[e2],[e3]])
        # e0_dot = e_dot.item(0)
        # e1_dot = e_dot.item(1)
        # e2_dot = e_dot.item(2)
        # e3_dot = e_dot.item(3)

        # rotatonal dynamics
        p_dot = MAV.gamma1 * p * q - MAV.gamma2 * q * r + MAV.gamma3 * l + MAV.gamma4 * n
        q_dot = MAV.gamma5 * p * r - MAV.gamma6 * (p ** 2 - r ** 2) + m / MAV.Jy
        r_dot = MAV.gamma7 * p * q - MAV.gamma1 * q * r + MAV.gamma4 * l + MAV.gamma8 * n

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind ):
        # def _update_velocity_data(self, wind):
        R = Quaternion2Rotation(self._state[6:10])
        # Should _wind be in inertial or body
        # pdb.set_trace()
        wind2 = R.T @ wind[0:3] + wind[3:6]
        self._wind = R @ wind2

        # wind2 = np.zeros((6, 1))

        self.ur = self._state.item(3) - wind2.item(0)  # + int(np.random.normal(2,2,1))
        self.vr = self._state.item(4) - wind2.item(1)  # + int(np.random.normal(2,2,1))
        self.wr = self._state.item(5) - wind2.item(2)  # + int(np.random.normal(2,2,1))
        #print("ur vr wr ", self.ur, self.vr, self.wr)
        # ur = self._state.item(3) - self._wind.item(0)
        # vr = self._state.item(4) - self._wind.item(1)
        # wr = self._state.item(5) - self._wind.item(2)

        # compute groud speed
        Vg = R @ self._state[3:6]
        self._Vg = np.sqrt(Vg.item(0) ** 2 + Vg.item(1) ** 2 + Vg.item(2) ** 2)
        self._chi = np.arctan2(Vg.item(1), Vg.item(0))
        if self._Vg == 0:
            self._gamma = MAV.psi0
        else:
            self._gamma = np.arcsin(Vg.item(2) / self._Vg)
        # compute airspeed
        self._Va = np.sqrt(self.ur ** 2 + self.vr ** 2 + self.wr ** 2)
        # compute angle of attack
        self._alpha = np.arctan2(self.wr, self.ur)
        # compute sideslip angle
        if self._Va == 0:
            self._beta = 0
        else:
            self._beta = np.arcsin(self.vr / self._Va)


    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        # assert delta.shape == (4,1)
        # de = delta[0,0]
        # dt = delta[1,0]
        # da = delta[2,0]
        # dr = delta[3,0]
        de = delta[1,0]
        dt = delta[3,0]
        da = delta[0,0]
        dr = delta[2,0]

        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)



        Fg = MAV.mass*MAV.gravity*np.array([[2*(e1*e3-e2*e0)],
                                            [2*(e2*e3 + e1*e0)],
                                            [e3**2+e0**2-e1**2-e2**2],
                                            ])

        M_e = 25
        sig = lambda a: (1+np.exp(-M_e*(a-MAV.alpha0))+np.exp(M_e*(a+MAV.alpha0)))/((1+np.exp(-M_e*(a-MAV.alpha0)))*(1+np.exp(M_e*(a+MAV.alpha0))))
        cla = lambda a: (1-sig(a))*(MAV.C_L_0+MAV.C_L_alpha*a)+sig(a)*(2*np.sign(a)*np.sin(a)**2*np.cos(a))
        cda = lambda a: MAV.C_D_p + (MAV.C_L_0+MAV.C_L_alpha*a)**2/(np.pi*MAV.e*MAV.AR)
        cxa = lambda a: -(cda(a)) * np.cos(a) + (cla(a)) * np.sin(a)

        cxq = lambda a: -MAV.C_D_q * np.cos(a) + MAV.C_L_q * np.sin(a)

        cxde = lambda a: -MAV.C_D_delta_e * np.cos(a) + MAV.C_L_delta_e * np.sin(a)

        cza = lambda a: -(cda(a)) * np.sin(a) - (cla(a)) * np.cos(a)

        czq = lambda a: -MAV.C_D_q * np.sin(a) - MAV.C_L_q * np.cos(a)

        czde = lambda a: -MAV.C_D_delta_e * np.sin(a) - MAV.C_L_delta_e * np.cos(a)

        c = MAV.c/(2*self._Va)
        b = MAV.b/(2*self._Va)

        Fa = 0.5*MAV.rho*self._Va**2*MAV.S_wing*np.array([\
            [1,0,0],[0,1,0],[0,0,1]]).dot(np.array([[cxa(self._alpha)+cxq(self._alpha)*c*q+cxde(self._alpha)*de],
            [MAV.C_Y_0+MAV.C_Y_beta*self._beta+MAV.C_Y_p*b*p+MAV.C_Y_r*b*r+MAV.C_Y_delta_a*da+MAV.C_Y_delta_r*dr],
            [cza(self._alpha)+czq(self._alpha)*c*q+czde(self._alpha)*de],
            ]))

        F = Fg + Fa
        #
        # print("Fa:",Fa)

        Fp = 0.5*MAV.rho*MAV.S_prop*MAV.C_prop*((MAV.k_motor*dt)**2-self._Va**2)

        # print("FP:", Fp)

        fx = F.item(0)\
            + Fp\
            # + 0.5*MAV.rho*self._Va**2*MAV.S_wing*(\
            #     +cxa(self._alpha)\
            #     + cxq(self._alpha)*c*q\
            #     + cxde(self._alpha)*de
            #     )

        fy = F.item(1)
        fz = F.item(2)

        #  Moment time!!!
        Ma = 0.5*MAV.rho*self._Va**2*MAV.S_wing*np.array([\
            [MAV.b*(MAV.C_ell_0+MAV.C_ell_beta*self._beta+MAV.C_ell_p*b*p+MAV.C_ell_r*b*r+MAV.C_ell_delta_a*da+MAV.C_ell_delta_r*dr)],
            [MAV.c*(MAV.C_m_0+(MAV.C_m_alpha*self._alpha)+(MAV.C_m_q*c*q)+(MAV.C_m_delta_e*de))],
            [MAV.b*(MAV.C_n_0+(MAV.C_n_beta*self._beta)+(MAV.C_n_p*b*p)+(MAV.C_n_r*b*r)+(MAV.C_n_delta_a*da)+(MAV.C_n_delta_r*dr))]
            ])
        # print("\nMa:", Ma)
        Mp = np.array([[-MAV.kTp*(MAV.kOmega*dt)**2],
                       [0.],
                       [0.]
                       ])

        M = Mp + Ma

        Mx = M.item(0)
        My = M.item(1)
        Mz = M.item(2)

        # self._forces[0] = fx
        # self._forces[1] = fy
        # self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _update_msg_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.h = -self._state.item(2)
        self.msg_true_state.Va = self._Va
        self.msg_true_state.alpha = self._alpha
        self.msg_true_state.beta = self._beta
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.Vg = self._Vg
        self.msg_true_state.gamma = self._gamma
        self.msg_true_state.chi = self._chi
        self.msg_true_state.p = self._state.item(10)
        self.msg_true_state.q = self._state.item(11)
        self.msg_true_state.r = self._state.item(12)
        self.msg_true_state.wn = self._wind.item(0)
        self.msg_true_state.we = self._wind.item(1)
        self.msg_true_state.ug = self._state.item(3)
        self.msg_true_state.vg = self._state.item(4)
        self.msg_true_state.wg = self._state.item(5)
        self.msg_true_state.ur = self.ur
        self.msg_true_state.vr = self.vr
        self.msg_true_state.wr = self.wr

    def _prop_force_moment_calc(self, delta_t):
        V_in = MAV.V_max*delta_t
        a = MAV.C_Q0*MAV.Made_up

    def reset(self):
        self.__init__(self._ts_simulation)
        return

