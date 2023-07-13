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
from tools.angleConversions import Quaternion2Euler, Quaternion2Rotation
from chap4.mav_dynamics import mav_dynamics
import parameters.simulation_parameters as SIM
import math as mt
from scipy import interpolate
from copy import deepcopy
from gym.spaces import Box
import gym
from typing import Tuple
import rlHelper.properties as prp
from rlHelper.properties import BoundedProperty
from chap6.autopilot import autopilot
from chap6.mat import A_lon_cont, B_lon_cont, trim_state, long_trim_state, trim_input

class rlWrapper:
    def __init__(self, perAgentStep, ts_simulation, trim_input, A_lon_cont, B_lon_cont, msg_trim_states ,discountfactor):

        self.PerAgentStep = perAgentStep
        self.MAVobject = self.__initMavObject()

        self.action_variables = (prp.Q0, prp.Q1, prp.Q2, prp.Q3, prp.Q4, prp.R1, prp.R2)
        self.state_variables = (prp.u_fps, prp.w_fps, prp.q_radps, prp.pitch_rad, prp.altitude_sl_ft)
        self.observation_space: gym.spaces.Box = self.get_state_space()
        self.action_space: gym.spaces.Box = self.get_action_space()
        self.deltaInput = trim_input
        # self.state = self.returnState()

        self.ctrl = autopilot(ts_simulation, trim_input, A_lon_cont, B_lon_cont, msg_trim_states)
        self.trimInput = trim_input # matlab no fixed initial
        self.prevError = 0
        self.discountfactor = discountfactor
        self.costavg =0
        self.costmin = 0
        self.costitr =1
        return

    def __reward(self, current_obs, next_obs, desiredState, input):

        cost_n = (current_obs.h - desiredState.h)**2  + (current_obs.ur - desiredState.ur)**2 + (current_obs.wr - desiredState.wr)**2 + (current_obs.q - desiredState.q)**2  + (current_obs.theta - desiredState.theta)**2
        cost_n = cost_n**(0.5)
        self.costitr+=1
        self.costavg = self.costavg + (cost_n - self.costavg)/(self.costitr)
        if self.costmin > cost_n:
            self.costmin = cost_n
        reward = min(max(0,(self.costavg-cost_n)/(self.costavg - self.costmin)),1)


        """reward = abs(desiredState.h - next_obs.h) / 5
        self.prevError = abs(desiredState.h - current_obs.h)
        heightreachreward = reward / (1 + reward)
        delError =0
        # print("current", current_obs.h , "nextobs", next_obs.h)
        #delError = (self.prevError - abs(desiredState.h - next_obs.h))
        #delError = delError / (delError + 1)
        # print('elError %.4f' % delError, 'reward %.4f' % heightreachreward)
        reward = 1 - heightreachreward"""
        return reward

    def __initMavObject(self):
        mav = mav_dynamics(SIM.ts_simulation)
        return mav

    def __ifdone(self, next_obs, i, desiredState):
        # print("next_obs.h - desiredState.h",next_obs.h , desiredState.h)
        if abs(next_obs.h - desiredState.h) >= 20:
            done = True
        else:
            done = False
        return done

    def returnState(self):

        currentState = deepcopy(self.MAVobject.msg_true_state)

        return currentState

    def step(self, delta2, desiredState, wind=np.zeros((6, 1))):
        # pdb.set_trace()
        for i in range(self.PerAgentStep):
            current_obs = self.returnState()
            self.MAVobject.update_state(delta2, wind)

            next_obs = self.returnState()

            # print("obs=",next_obs.h)
            done = self.__ifdone(next_obs, i, desiredState)
            if done:
                break

        reward = self.__reward(current_obs, next_obs, desiredState, delta2)

        state = next_obs
        state = np.array([state.ur, state.wr, state.q, state.theta, state.h])
        return deepcopy(state), reward, done

    def step2(self, action, desiredState, commands):
        estimated_state = self.returnState()
        delta, commanded_state = self.ctrl.update2(commands, estimated_state, self.deltaInput, action)
        self.deltaInput = delta
        delta2 = delta + self.trimInput

        return self.step(delta2, desiredState)

    def step3(self, action, desiredState ,cmdState):
        done = False
        total_reward = 0
        i = 0
        while not done:
            estimated_state = self.returnState()
            delta, commanded_state, flag = self.ctrl.update2(cmdState, estimated_state, self.deltaInput, action)
            #pdb.set_trace()
            if ~flag:
                self.deltaInput = delta
                delta2 = delta + self.trimInput
                state, reward, done = self.step(delta2, desiredState)
                total_reward += self.discountfactor*reward
                i += 1
                
            if flag:
                done = True
                total_reward = 0
            done = done or i == 300

        total_reward = total_reward / (i + 10e-10)

        if total_reward < 0:
            total_reward = 0
        print("i=", i)
        return state, total_reward, done

    def reset(self):
        #pdb.set_trace()
        self.MAVobject.reset()
        current_obs = self.returnState()
        state = current_obs
        state = np.array([state.ur, state.wr, state.q, state.theta, state.h])
        self.deltaInput = self.trimInput
        self.prevError = 0
        self.costitr =1
        self.costavg =0
        self.costmin = 0
        return state

    def get_state_space(self) -> gym.Space:
        state_lows = np.array([state_var.min for state_var in self.state_variables])
        state_highs = np.array([state_var.max for state_var in self.state_variables])
        return gym.spaces.Box(low=state_lows, high=state_highs, dtype='float')

    def get_action_space(self) -> gym.Space:
        action_lows = np.array([act_var.min for act_var in self.action_variables])
        action_highs = np.array([act_var.max for act_var in self.action_variables])
        return gym.spaces.Box(low=action_lows, high=action_highs, dtype='float')
