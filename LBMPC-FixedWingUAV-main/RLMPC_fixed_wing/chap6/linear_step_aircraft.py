import numpy as np
import pdb
from casadi import*

from chap6.mat import A_lon_cont,B_lon_cont
dt = 0.1

fxdot = lambda x, u: np.matmul(A_lon_cont, x) + np.matmul(B_lon_cont, u)  # x_dot=AX+BU


def next_step( state_current,input_current):
    #pdb.set_trace()
    k1 = fxdot(state_current, input_current)
    k2 = fxdot(state_current + dt / 2 * k1, input_current)
    k3 = fxdot(state_current + dt / 2 * k2, input_current)
    k4 = fxdot(state_current + dt * k3, input_current)
    x_next = state_current + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    #print("x0==", x_next)
    #print("out")
    return x_next



if __name__ == "__main__":
    next_step(np.random.rand(5,1),np.random.rand(3,1))