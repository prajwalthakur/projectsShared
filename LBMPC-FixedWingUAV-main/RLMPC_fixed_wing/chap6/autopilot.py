"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np

import os
sys.path.insert(1,os.getcwd())
import parameters.control_parameters as AP

from tools.transfer_function import transfer_function
from chap6.pid_control import pid_control, pi_control, pd_control_with_rate
from message_types.msg_state import msg_state
from chap6.linear_mpc import longitudinal_mpc

import pdb


class autopilot:
    def __init__(self, ts_control, initial_trim_input, A_lon, B_lon, trim_state):
        # instantiate lateral controllers
        self.trim_state = trim_state
        self.roll_from_aileron = pd_control_with_rate(
            kp=AP.roll_kp,
            kd=AP.roll_kd,
            limit=np.radians(45))
        self.course_from_roll = pi_control(
            kp=AP.course_kp,
            ki=AP.course_ki,
            Ts=ts_control,
            limit=np.radians(30))
        self.sideslip_from_rudder = pi_control(
            kp=AP.sideslip_kp,
            ki=AP.sideslip_ki,
            Ts=ts_control,
            limit=np.radians(45))
        self.yaw_damper = transfer_function(
            num=np.array([[AP.yaw_damper_kp, 0]]),
            den=np.array([[1, 1 / AP.yaw_damper_tau_r]]),
            Ts=ts_control)
        self.trim_input = initial_trim_input
        self.Ts = ts_control

        # instantiate longitudinal controllers
        self.pitch_from_elevator = pd_control_with_rate(
            kp=AP.pitch_kp,
            kd=AP.pitch_kd,
            limit=np.radians(45))
        self.altitude_from_pitch = pi_control(
            kp=AP.altitude_kp,
            ki=AP.altitude_ki,
            Ts=ts_control,
            limit=np.radians(30))
        self.airspeed_from_throttle = pi_control(
            kp=AP.airspeed_throttle_kp,
            ki=AP.airspeed_throttle_ki,
            Ts=ts_control,
            limit=1.0)
        self.commanded_state = msg_state()
        self.longi_mpc = longitudinal_mpc(
            A_longitudinal=A_lon,
            B_longitudinal=B_lon
        )

    def update2(self, cmd, state, current_del_input ,weights):
        #pdb.set_trace()
        # lateral autopilot
        course_wrapped = self.wrap(state.chi, cmd.course_command)
        phi_c = self.course_from_roll.update(course_wrapped, state.chi)
        # phi_c = np.radians(0)
        phi_c += cmd.phi_feedforward
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        delta_r = self.yaw_damper.update(state.r)
        # delta_a =0
        # delta_r =0
        # longitudinal autopilot
        # MPC CONTROLLER FOR LONGITUDINAL D Y A N M I C S

        h_c = cmd.altitude_command
        [theta_c, delta_e, delta_t ,ExceptionFlag] = self.longi_mpc.update(h_c - self.trim_state.h,
                                                                                               self.trim_state, state,
                                                                                               current_del_input,
                                                                                               cmd.airspeed_command - self.trim_state.ur,
                                                                                               weights)

        delta = np.array([[delta_a], [delta_e], [delta_r], [delta_t] ])
        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state , ExceptionFlag

    def saturate(self, inputs, low_limits, up_limits):
        i = 0
        for input, low_limit, up_limit in zip(inputs, low_limits, up_limits):

            if input <= low_limit:
                output = low_limit
            elif input >= up_limit:
                output = up_limit
            else:
                output = input
            inputs[i] = output
            i += 1
        return inputs

    def wrap(self, current, input):
        while (input - current) > np.pi:
            input -= 2 * np.pi
        while (input - current) < -np.pi:
            input += 2 * np.pi
        return input
