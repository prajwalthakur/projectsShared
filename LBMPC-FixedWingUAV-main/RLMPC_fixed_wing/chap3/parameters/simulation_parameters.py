import sys

import os
sys.path.insert(1,os.getcwd())
import numpy as np

######################################################################################
#   sample times, etc
######################################################################################
ts_simulation = 0.008 #0.001  # smallest time step for updating true non linear system runge kutta discretization
start_time = 0.  # start time for simulation
end_time = 250.  # end time for simulation

ts_plotting = 0.05  # refresh rate for plots

ts_control = 0.008  #0.01  # sample rate for the controller

# Wind parameters
Lu = 200
Lv = Lu
Lw = 50
sigma_u = 1.06
sigma_v = sigma_u
sigma_w = .7
