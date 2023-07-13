from casadi import *
import pdb
import numpy as np
import scipy.integrate
def del_lti_dyn_simple(A_disc,B_disc,T_s ,tau_u,tau_d ,x,u_1_k,u_k,const,G,H0,H1):
    # A = A_disc
    # B = B_disc
    # G = expm(A*T_s)
    # f = lambda t :expm(A*t) 
    # H1 = expm(A*(T_s-tau_d()-tau_u()))*scipy.integrate.solve_ivp(f , t_span=[0, tau_u() + tau_d()] ,y0 =0,t_eval=[T_s])*B
    # H0 = scipy.integrate.solve_ivp(f , t_span=[0,T_s- tau_u() - tau_d()] ,y0 =0,t_eval=[T_s])*B
    x_k_1= G@x  + H1@u_1_k + H0@u_k + const@T_s
    return x_k_1
class delayed_mpc:
    def __init__(self, ts_control ):
        self.index = 0
        self.Ts_control = ts_control  # runge kutta discretization of  continous state space model
        self.N = 18  # TODO INITIAL 18  # number of control intervals
        # optimization variable
        self.opti = Opti()  # Optimization problem
        # ---- decision variables ---------
        self.states = 3
        self.control_variables = 1
        self.X = self.opti.variable(self.states, self.N+1)  # state
        self.U = self.opti.variable(self.control_variables, self.N)  # control trajectory (throttle)
        self.state_current = self.opti.parameter(3) #initialize the states and control with last solution

        # Differential equation
        # xdot = [x_dot,y_dot,psi_dot]
        #u =[speed,steering_angle]
        self.A = self.opti.parameter(3,3)
        self.B = self.opti.parameter(3,1)
        self.constant = self.opti.parameter(3,1)
        self.tau_u =self.opti.parameter(1,1)
        self.tau_d = self.opti.parameter(1,1)
        self.H0 =self.opti.parameter(3,1)
        self.H1 = self.opti.parameter(3,1)
        self.G = self.opti._parameter(3,3)
        #self.fxdot = lambda x,u : mtimes(self.A, x) + mtimes(self.B, u)
        #self.fxdot = lambda x, u : vertcat(u[0] * cos(x[2]), u[0] * sin(x[2]), (u[0] / self.L) * tan(u[1]))  # dynamic equations of the states
        # weight and cost matrix
        self.Q = self.opti.parameter(3, 3)
        self.R = self.opti.parameter(1, 1)

        # subject_to giving reference set in update function
        self.x_ref = self.opti.parameter()
        self.y_ref = self.opti.parameter()
        self.psi_ref = self.opti.parameter()
        self.steering_last_command = self.opti.parameter()
        
        ############constraints on speed
        # self.v_max = 0.6
        # self.v_min = -self.v_max
        self.delta_max = pi / 6   ##7 degree
        self.delta_min = -self.delta_max
        
        # output reference
        self.output_ref = vertcat(self.x_ref, self.y_ref,self.psi_ref)

        # control  bound
        # self.opti.subject_to(self.opti.bounded(self.v_min, self.U[0, :],
        #                                        self.v_max))  # speed limit
        self.opti.subject_to(self.opti.bounded(self.delta_min, self.U[0, :],
                                                self.delta_max))  # steering limit

        # state bound  TODO: RELAXING FOR NOW
        # self.opti.subject_to(self.opti.bounded(-60 * pi / 180, self.X[3, :],
        #                                        (60 * pi / 180)))  # state : theta is bound

        self.dt = self.Ts_control
        self.obj = 0  # Objective
        # for k in range(0, self.N, 1):
        #     con = self.U[:, k]
        #     k1 = self.fxdot(self.X[:, k ], self.U[:, k])
        #     k2 = self.fxdot(self.X[:, k ] + self.dt / 2 * k1, self.U[:, k])
        #     k3 = self.fxdot(self.X[:, k ] + self.dt / 2 * k2, self.U[:, k])
        #     k4 = self.fxdot(self.X[:, k ] + self.dt * k3, self.U[:, k])
        #     x_next = self.X[:, k ] + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        #     self.X[:, k+1] = x_next
        #     self.obj = self.obj + mtimes(mtimes((self.X[:, k ] - self.y_ref).T, self.Q),
        #                                  (self.X[:, k ] - self.y_ref)) + \
        #                mtimes(mtimes(con.T, self.R), con)
        #     self.opti.subject_to(self.X[:,k+1]==x_next)
        #pdb.set_trace()
        self.obj = self.obj  + (self.X[:, 0 ] - self.output_ref).T@self.Q@(self.X[:, 0 ] - self.output_ref) + self.U[:, 0].T@self.R@self.U[:, 0] 
        x_next = del_lti_dyn_simple(self.A,self.B,self.Ts_control ,self.tau_u,self.tau_d,self.X[:,0],self.steering_last_command,self.U[:,0],self.constant,self.G,self.H0,self.H1)
        self.opti.subject_to(self.X[:,1]==x_next)
        for k in range(1, self.N, 1):
            self.obj = self.obj + (self.X[:, k ] - self.y_ref).T@self.Q@(self.X[:, k ] - self.y_ref) + self.U[:,k].T@self.R@self.U[:,k]
            x_next = del_lti_dyn_simple(self.A,self.B,self.Ts_control ,self.tau_u,self.tau_d ,self.X[:,k],self.U[:,k-1],self.U[:,k],self.constant,self.G,self.H0,self.H1)
            self.opti.subject_to(self.X[:,k+1]==x_next)


        self.opti.subject_to(self.X[:, 0] == self.state_current[0:3])
        #self.opti.set_initial(self.U[:,0],self.steering_last_command)
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
            "ipopt.print_level": 3,
            "print_time": False
            # "hessian_approximation": "exact"
        }

        self.opti.solver('ipopt', ipopt_options)
        
        self.opti.minimize(self.obj)

    def update(self, x_ref, y_ref , psi_ref, state_current, input_last, weights,v_cosnt,l_f,tau_u,tau_d):
        #pdb.set_trace()
        # print("href" , h_ref,"uref", u_ref)
        self.opti.set_value(self.state_current[0], state_current[0] )
        self.opti.set_value(self.state_current[1], state_current[1])
        self.opti.set_value(self.state_current[2], state_current[2] )
        self.opti.set_value(self.x_ref, x_ref)
        self.opti.set_value(self.y_ref, y_ref)
        self.opti.set_value(self.psi_ref, psi_ref)
        self.opti.set_value(self.steering_last_command,input_last)
        self.opti.set_value(self.tau_u,tau_u)
        self.opti.set_value(self.tau_d,tau_d) 
        self._updateWeights(weights)
        self._updateSystemMatrix(v_cosnt,l_f,self.Ts_control,tau_u,tau_d)
        try:
            #pdb.set_trace()
            sol = self.opti.solve()
            input_commands = sol.value(self.U[:,0:])
            flag = 0
            return input_commands, flag
        except:
            flag = 1
            return -1, flag

    def _updateWeights(self, weights):
        Q = np.zeros((3, 3))  # for state error
        Q[0, 0] = weights[0]
        Q[1, 1] = weights[1]
        Q[2, 2] = weights[2]
        # control cost matrix
        R = np.zeros((1, 1))
        R[0, 0] = weights[3]
        self.opti.set_value(self.Q, Q)
        self.opti.set_value(self.R, R)
        return
    # def _get_matrices_for_LTI(self,tau_u,tau_d)
    #     A = A_disc
    #     B = B_disc
    #     G = expm(A*T_s)
    #     f = lambda t :expm(A*t) 
    #     H1 = expm(A*(T_s-tau_d()-tau_u()))*scipy.integrate.solve_ivp(f , t_span=[0, tau_u() + tau_d()] ,y0 =0,t_eval=[T_s])*B
    #     H0 = scipy.integrate.solve_ivp(f , t_span=[0,T_s- tau_u() - tau_d()] ,y0 =0,t_eval=[T_s])*B
    def _func(self,t,y,A):
        y = scipy.linalg.expm(A*t).reshape((9,))
        return y
    def _updateSystemMatrix(self,v_cosnt,l_f,T_s,tau_u,tau_d):
        A = np.asarray([[0,0,0],[0,0,v_cosnt],[0,0,0]])
        B = np.asarray([[0],[0],[v_cosnt/l_f]])
        G = scipy.linalg.expm(A*T_s)
        H1 = scipy.linalg.expm(A*(T_s-tau_d-tau_u))@scipy.integrate.solve_ivp(self._func  , t_span=(0, tau_u+ tau_d) ,y0 =np.zeros((9,)),t_eval=[tau_u+ tau_d],args = [A] ).y.reshape((3,3))@B
        H0 = scipy.integrate.solve_ivp(self._func , t_span=[0,T_s- tau_u - tau_d] ,y0 =np.zeros((9,)),t_eval=[T_s- tau_u - tau_d],args=[A]).y.reshape((3,3))@B
        self.opti.set_value(self.A, A)
        self.opti.set_value(self.B, B)
        self.opti.set_value(self.constant,np.asarray([[v_cosnt],[0],[0]]))
        self.opti.set_value(self.H0, H0)
        self.opti.set_value(self.H1,H1)
        self.opti.set_value(self.G,G)