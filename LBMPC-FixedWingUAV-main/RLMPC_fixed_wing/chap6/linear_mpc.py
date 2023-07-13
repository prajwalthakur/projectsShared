from casadi import *
import pdb
import numpy as np
from parameters.simulation_parameters import ts_control


class longitudinal_mpc:
    def __init__(self, A_longitudinal, B_longitudinal):
        self.index = 0
        self.Ts_control = ts_control  # runge kutta discretization of  continous state space model
        self.N = 18  # TODO INITIAL 18  # number of control intervals
        self.A_lon = A_longitudinal
        self.B_lon = B_longitudinal

        # optimization variable
        self.opti = Opti()  # Optimization problem
        # ---- decision variables ---------
        self.states = 5
        self.control_variables = 2
        self.X = self.opti.variable(self.states, self.N)  # state
        self.U = self.opti.variable(self.control_variables, self.N)  # control trajectory (throttle)
        self.state_current = self.opti.parameter(7)

        # Differential equation
        # xdot = AX +BU

        self.fxdot = lambda x, u: mtimes(self.A_lon, x) + mtimes(self.B_lon, u)  # x_dot=AX+BU

        # weight and cost matrix

        """self.Q = np.zeros((5, 5))  # for state error
        self.Q[0, 0] = 1.0
        self.Q[1, 1] = .01
        self.Q[2, 2] = .5
        self.Q[3, 3] = 2.001
        self.Q[4, 4] = 5.0
        # control cost matrix
        self.R = np.zeros((3, 3))
        self.R[0, 0] = 4.00
        self.R[1, 1] = 0.001
        self.R[2, 2] = 0.001"""
        self.Q = self.opti.parameter(5, 5)
        self.R = self.opti.parameter(2, 2)

        # subject_to giving reference set in update function
        self.h_ref = self.opti.parameter()
        self.u_ref = self.opti.parameter()
        self.w_ref = self.opti.parameter()
        self.q_ref = self.opti.parameter()
        self.theta_ref = self.opti.parameter()
        # output reference
        self.y_ref = vertcat(self.u_ref, self.w_ref, self.q_ref, self.theta_ref, self.h_ref)

        # control  bound
        self.opti.subject_to(self.opti.bounded(-60 * pi / 180, self.U[0, :],
                                               60 * pi / 180))  # track speed limit
        self.opti.subject_to(self.opti.bounded(-3.14346798e-01, self.U[1, :],
                                               1 - 3.14346798e-01))  # thrust control is limited
        # self.opti.subject_to(self.opti.bounded(-6, self.U[2, :],
        #                                      20))  # thrust control is

        # state bound
        self.opti.subject_to(self.opti.bounded(-60 * pi / 180, self.X[3, :],
                                               (60 * pi / 180)))  # state : theta is bound

        self.dt = self.Ts_control
        self.obj = 0  # Objective
        for k in range(1, self.N, 1):
            con = self.U[:, k]

            k1 = self.fxdot(self.X[:, k - 1], self.U[:, k])
            k2 = self.fxdot(self.X[:, k - 1] + self.dt / 2 * k1, self.U[:, k])
            k3 = self.fxdot(self.X[:, k - 1] + self.dt / 2 * k2, self.U[:, k])
            k4 = self.fxdot(self.X[:, k - 1] + self.dt * k3, self.U[:, k])
            x_next = self.X[:, k - 1] + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self.X[:, k] = x_next
            self.obj = self.obj + mtimes(mtimes((self.X[:, k - 1] - self.y_ref).T, self.Q),
                                         (self.X[:, k - 1] - self.y_ref)) + \
                       mtimes(mtimes(con.T, self.R), con)

        self.opti.subject_to(self.X[:, 0] == self.state_current[0:5])
        self.opti.subject_to(self.U[:, 0] == self.state_current[5:])

        # self.opti.subject_to(self.Vo)

        self.opti.minimize(self.obj)
        prnt = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        # options = {"ipopt": {"hessian_approximation": "exact"}, 'ipopt.acceptable_tol': 1e-1}
        options = {"ipopt": {"hessian_approximation": "exact"}, 'ipopt.acceptable_tol': 1e-4
            , 'ipopt.print_level': 1, 'print_time': 0, 'ipopt.sb': 'yes'}
        ipopt_options = {
            'verbose': False,
            "ipopt.tol": 1e-8,
            "ipopt.acceptable_tol": 1e-6,
            "ipopt.max_iter": 100,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.print_level": 0,
            "print_time": False
            # "hessian_approximation": "exact"
        }

        self.opti.solver('ipopt', ipopt_options)

    def update(self, h_ref, trim_state, state_current, del_input_current, u_ref, weights):
        #pdb.set_trace()
        # print("href" , h_ref,"uref", u_ref)
        self.opti.set_value(self.state_current[0], state_current.ur - trim_state.ur)
        self.opti.set_value(self.state_current[1], state_current.wr - trim_state.wr)
        self.opti.set_value(self.state_current[2], state_current.q - trim_state.q)
        self.opti.set_value(self.state_current[3], state_current.theta - trim_state.theta)
        self.opti.set_value(self.state_current[4], state_current.h - trim_state.h)
        self.opti.set_value(self.state_current[5], (del_input_current[1][0]))
        self.opti.set_value(self.state_current[6], (del_input_current[3][0]))
        ref = (np.array([[u_ref], [0], [0], [0.0], [h_ref]]))
        # ref = np.array([[u_ref], [0], [0], [0], [h_ref]])
        self.opti.set_value(self.h_ref, ref[4][0])
        self.opti.set_value(self.u_ref, ref[0][0])
        self.opti.set_value(self.w_ref, ref[1][0])  # - trim_state.wr)
        self.opti.set_value(self.q_ref, ref[2][0])
        self.opti.set_value(self.theta_ref, ref[3][0])  # -trim_state.theta)
        self._updateWeights(weights)
        try:
            sol = self.opti.solve()
            # pdb.set_trace()
            [theta, delta_e, delta_t] = [sol.value(self.X[3, 1]), sol.value(self.U[0, 1]),
                                         sol.value(self.U[1, 1])]

            # print("del_height=", sol.value(self.X[4, 1]))
            # self.index+=1
            # print(self)
            flag = 0
            return theta, delta_e, delta_t, flag
        except:
            flag = 1
            return -1, -1, -1, flag

    def _updateWeights(self, weights):
        Q = np.zeros((5, 5))  # for state error
        Q[0, 0] = weights[0]
        Q[1, 1] = weights[1]
        Q[2, 2] = weights[2]
        Q[3, 3] = weights[3]
        Q[4, 4] = weights[4]
        # control cost matrix
        R = np.zeros((2, 2))
        R[0, 0] = weights[5]
        R[1, 1] = weights[6]
        self.opti.set_value(self.Q, Q)
        self.opti.set_value(self.R, R)

        return
