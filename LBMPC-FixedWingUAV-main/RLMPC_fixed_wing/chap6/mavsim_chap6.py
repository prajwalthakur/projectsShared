import sys
import pdb
import os
sys.path.insert(1,os.getcwd())
#pdb.set_trace()
import parameters.simulation_parameters as SIM  # parameters.simulation_parameters as SIM
from chap4.mav_dynamics import mav_dynamics
from tools.angleConversions import Euler2Quaternion
from chap6.autopilot import autopilot
from tools.signals import signals
from copy import deepcopy
from casadi import *
from chap6.linear_step_aircraft import next_step
#from chap3.dataViewerDebug import data_viewerdebug
#from chap3.data_viewer import data_viewer
from timeit import default_timer as timer
import random
from message_types.msg_autopilot import msg_autopilot
from chap6.mat import A_lon_cont, B_lon_cont, trim_state, long_trim_state, trim_input
import pdb

# loading  trims value
Va = 25.00
gamma = 0. * np.pi / 180.


weights = np.array([1.0, 0.1, 0.5, 2.001, 5.0, 4.00, .01])
""""[ 0.19010967  5.1139468   4.82674149  5.26969916  5.31558227 -0.29457227
  5.4555894   5.24866493]",0.6217586157626882"""

"""""[ 0.30611667  5.02722512  4.83456607  4.81023379  4.95270451 -0.11439833 5.15594659  4.95798248],0.617883149112560"""""

""""[2.55460801 2.49933617 2.52168748 2.69877644 2.59287848 2.58079338 2.50337014 2.54272376]",0.49121906413479727"""
# run 2
""""[-0.14200419  0.05473975 10.27344833  1.17726456  1.47487805  9.8929652
  9.89030071  3.94785597], 0.0203253012604195"""
""""[5.53556456 4.91154139 5.10360977 4.94541378 4.98149314 4.88813042
 4.85286054 4.97834812]",0.002579066658444551"""


#weights = np.array([2.55460801, 2.49933617 ,2.52168748 ,2.69877644 ,2.59287848 ,2.58079338 ,2.50337014 ,2.54272376])

# weights = np.array([ 0.30611667 , 5.02722512  ,4.83456607  ,4.81023379,  4.95270451 ,-0.11439833 ,5.15594659,  4.95798248])

def linearMPC(delta, trim_input, mav):
    v = 0
    sim_time = SIM.start_time
    commands = msg_autopilot()
    Va_command = signals(dc_offset=25, amplitude=0.0, start_time=0.0, frequency=0.005)  # TODO INITIAL +3.0
    # Va_command = signals(dc_offset=28.0) #amplitude=3.0, start_time=2.0, frequency = 0.01)
    h_command = signals(dc_offset=100.0, amplitude=-10.0, start_time=0.0, frequency=0.00002)  # TODO INITIAL -3.0
    msg_trim_states = deepcopy(mav.msg_true_state)
    ctrl = autopilot(SIM.ts_simulation, trim_input, A_lon_cont, B_lon_cont, msg_trim_states)
    start = timer()
    while sim_time < SIM.end_time:


        v += 1
        random.seed(timer())
        # -------controller-------------
        estimated_state = mav.msg_true_state  # uses true states in the control
        commands.airspeed_command = Va_command.square(sim_time)
        commands.altitude_command = h_command.square(sim_time)
        delta, commanded_state, flag = ctrl.update2(commands, estimated_state, delta, weights)

        current_wind = np.zeros((6, 1))

        delta2 = trim_input + delta
        # pdb.set_trace()
        previous_states = deepcopy(mav.msg_true_state)
        print("iter=", v, "height=",previous_states.h)
        sim_time += SIM.ts_simulation
        mav.update_state(delta2, current_wind)
        x_longitudinal_nlp = np.array([
            [previous_states.ur],  # (3)
            [previous_states.wr],  # (5)
            [previous_states.q],  # (10)
            [previous_states.theta],  # (11)
            [previous_states.h]])
        # pdb.set_trace()
        x_linear_deviation = x_longitudinal_nlp - long_trim_state
        # x_linear_deviation = x_linear_prediction - long_trim_state
        x_linear_prediction = next_step(x_linear_deviation, np.array([
            [delta[1][0]],
            [delta[3][0]],
        ])) + long_trim_state
        """datadebug_view.update(mav.msg_true_state,  # true states
                          x_linear_prediction,  # linear_estimated states
                          commands,  # commanded states
                          delta2,
                          sim_time)
        data_view.update(mav.msg_true_state,  # true states
                          mav.msg_true_state,  # estimated states
                          commands,  # commanded states
                          delta2,
                          sim_time)"""
    return


if __name__ == "__main__":
    #datadebug_view = data_viewerdebug()
    #data_view = data_viewer()
    mav = mav_dynamics(SIM.ts_simulation)
    mav._state = trim_state  # set the initial state of the mav to the trim state
    delta = trim_input  # set initial  input to constant constant trim input

    # openLoop()

    linearMPC(delta, trim_input, mav)
