import sys

import os
sys.path.insert(1,os.getcwd())
import numpy as np
import parameters.simulation_parameters as SIM  # parameters.simulation_parameters as SIM

from chap4.wind_simulation import wind_simulation
from tools.angleConversions import Euler2Quaternion
from tools.signals import signals
from copy import deepcopy
from chap6.linear_step_aircraft import next_step
from chap3.dataViewerDebug import data_viewerdebug
from timeit import default_timer as timer
import random
from message_types.msg_autopilot import msg_autopilot
from rlHelper.rl import rlWrapper
from message_types.msg_state import msg_state
import pdb
from chap6.mat import A_lon_cont, B_lon_cont, trim_state, long_trim_state, trim_input
# loading  trims value
Va = 25.00
gamma = 0. * np.pi / 180.
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


def Openloop(delta, trim_input):
    sim_time = SIM.start_time
    pi = np.pi
    commands = msg_autopilot()
    desiredState = msg_state
    desiredState.h = 105
    Va_command = signals(dc_offset=25, amplitude=0.0, start_time=0.0, frequency=0.0000005)  # TODO INITIAL +3.0
    # Va_command = signals(dc_offset=28.0) #amplitude=3.0, start_time=2.0, frequency = 0.01)
    h_command = signals(dc_offset=100.0, amplitude=0.0, start_time=0.0, frequency=0.0000001)  # TODO INITIAL -3.0
    v = 0
    sim_time = SIM.start_time
    start = timer()
    perAgentStep = 10
    rlWrap = rlWrapper(perAgentStep,ts_simulation, trim_input, A_lon_cont, B_lon_cont, msg_trim_states ,discountfactor)
    while sim_time < SIM.end_time:

        commands.airspeed_command = Va_command.square(sim_time)
        commands.altitude_command = h_command.square(sim_time)
        print(v)
        v += 1
        random.seed(timer())

        current_wind = np.zeros((6, 1))

        # print("input ", delta2)
        # print("time=", (timer() - start) / 60)
        if (timer() - start) / 60 >= 0.050:  # after some time change the elevator to -6 degree
            # pdb.set_trace()
            # rlWrap.reset()
            trim_input[1][0] = -60 * pi / 180
        # print("timer=", timer())
        delta2 = trim_input  # + delta
        #  print(delta2)
        i = 0
        previous_states = deepcopy(rlWrap.returnState())
        sim_time += SIM.ts_simulation

        (obs,reward,done) = rlWrap.step(previous_states, delta2, current_wind , desiredState)

            # mav.update_state(delta2, current_wind)  # update the true non linear system
        # TODO check the previous state and the next updated state .....very small step size might be an issue
        # print("prev_h=", previous_states.h , "next_h=",mav.msg_true_state.h)
        # update the linear model from the previous non linear states
        x_longitudinal_nlp = np.array([
            [previous_states.ur],  # (3)
            [previous_states.wr],  # (5)
            [previous_states.q],  # (10)
            [previous_states.theta],  # (11)
            [previous_states.h]])
        x_deviation = x_longitudinal_nlp - long_trim_state
        # x_linear_deviation = x_linear_prediction - long_trim_state
        x_linear_prediction = next_step(x_deviation, np.array([
            [delta2[1][0]],  # (3)
            [delta2[3][0]],  # (5)
            [delta2[4][0]]
        ])) + long_trim_state
        # graph plotter
        datas_view.update( obs,  # true states
                          x_linear_prediction,  # linear_estimated states
                          commands,  # commanded states
                          delta2,
                          (timer() - start) / 60)
        sim_time += SIM.ts_simulation
        if done:
            pdb.set_trace()

    return


if __name__ == "__main__":
    #datas_view = data_viewerdebug()
    delta = trim_input  # set initial  input to constant constant trim input

    Openloop(delta, trim_input) #, datas_view)
