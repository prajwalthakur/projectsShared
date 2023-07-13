import numpy as np
import pdb
#from casadi import*

A_lon_cont = np.array([[-0.0475, 0.2383, -1.6000, -9.7900, 0],
                       [-0.6016, -2.6981, 24.9000, -0.6264, 0.],
                       [-0.0884, -0.5145, -0.8796, 0.0, 0.],
                       [0., 0.0, 1.0, 0.0, 0.],
                       [0.0639, -0.9980, 0.0, 25.000, 0.]])

B_lon_cont = np.array([[-0.3062, 0.0735, 0.0735],
                       [4.7643, -0.0087, -0.0087],
                       [-49.8341, -0.000, -0.000],
                       [0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0]])
time_step = 0.001
dt = 0.001

fxdot = lambda x, u: np.matmul(A_lon_cont, x) + np.matmul(B_lon_cont, u)  # x_dot=AX+BU


def next_step( state_current,input_current):
    #pdb.set_trace()
    # Integrate state space using Runge-Kutta RK4 algorithm
    k1 = fxdot(state_current, input_current)
    k2 = fxdot(state_current + dt / 2 * k1, input_current)
    k3 = fxdot(state_current + dt / 2 * k2, input_current)
    k4 = fxdot(state_current + dt * k3, input_current)
    x_next = state_current + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    return x_next



if __name__ == "__main__":
    next_step(np.random.rand(5,1),np.random.rand(3,1))