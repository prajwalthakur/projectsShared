import sys

import os
sys.path.insert(1,os.getcwd())
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
import pdb
import csv
import time

# loading  trims value
Va = 25.00
gamma = 0. * np.pi / 180.
from chap6.mat import A_lon_cont, B_lon_cont, trim_state, long_trim_state, trim_input

from rlHelper.DDPG.ddpf_tf import Agent
from rlHelper.DDPG.utils import plotLearning

# Uncomment the lines below to specify which gpu to run on
#os.environ["CUDA_DEVICE_ORDER"] = "0000:06:00.0"
#os.environ["CUDA_VISIBLE_DEVICES"] = "1"
import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
tf.config.experimental.set_visible_devices(devices=gpus[0], device_type='GPU')
"""tf.config.experimental.set_virtual_device_configuration(
    gpus[1],
    [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=2048)])"""
tf.config.experimental.set_memory_growth(gpus[0], False)
tf.config.experimental.VirtualDeviceConfiguration( memory_limit=2048)
from rlHelper.rl import rlWrapper
from message_types.msg_state import msg_state


def writeCSV(env,agent,desiredState, commands):
    obs = env.reset()
    act = agent.choose_action(obs)
    new_state, reward, done = env.step3(act, desiredState, commands)

    act = agent.choose_action(obs)
    with open('weight_data.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([new_state,act, reward])
    return

def linearMPC(delta, trim_input, mav, logdir):
    v = 0
    sim_time = SIM.start_time
    commands = msg_autopilot()
    #Va_command = signals(dc_offset=25, amplitude=0.0, start_time=0.0, frequency=0.00000000000000001)  # TODO INITIAL +3.0
    # Va_command = signals(dc_offset=28.0) #amplitude=3.0, start_time=2.0, frequency = 0.01)
    #h_command = signals(dc_offset=100.0, amplitude=-10.0, start_time=0.0, frequency=0.00000000000001)  # TODO INITIAL -3.0
    msg_trim_states = deepcopy(mav.msg_true_state)
    start = timer()
    discountfactor = 0.9999
    env = rlWrapper(1, SIM.ts_simulation, trim_input, A_lon_cont, B_lon_cont, msg_trim_states, discountfactor)
    agent = Agent(alpha=0.0005, beta=0.0005, input_dims=[5], tau=0.001,
                  env=env, logdir=logdir, batch_size=62, layer1_size=124, layer2_size=62,
                  n_actions=7)

    np.random.seed(0)
    score_history = []
    desiredState = deepcopy(mav.msg_true_state)

    #pdb.set_trace()
    j = 0
    commands.airspeed_command = 25 #Va_command.square(sim_time)
    commands.altitude_command = 90 #h_command.square(sim_time)
    desiredState.h = 90 #h_command.square(sim_time)
    desiredState.ur = 25  #Va_command.square(sim_time)
    desiredState.wr = 0
    desiredState.q = 0
    desiredState.theta = 0
    writeCSV(env,agent,desiredState, commands)
    flagitr =1
    for i in range(2000000):
        print("episode=",i)
        obs = env.reset()
        done = False
        score = 0

        flaglearn = 0

        while not done:
            act = agent.choose_action(obs)
            new_state, reward, done = env.step3(act, desiredState, commands)
            # pdb.set_trace()
            agent.remember(obs, act, reward, new_state, int(done))

            flaglearn = agent.learn()
            score += reward
            obs = new_state
            # env.render()
        score_history.append(score)
        print('episode ', i, 'score %.4f' % score, "height", new_state[4],
              'trailing 100 games avg %.5f' % np.mean(score_history[-100:]))
        #pdb.set_trace()
        if flaglearn:
            print("testing learned actor number=",flagitr)
            flagitr+=1
            writeCSV(env,agent,desiredState, commands)
    filename = 'Pendulum-alpha0-0005-beta0-0005-124-62-optimized.png'
    plotLearning(score_history, filename, window=100)
    return


if __name__ == "__main__":
    #datas_view = data_viewerdebug()
    mav = mav_dynamics(SIM.ts_simulation)
    mav._state = trim_state  # set the initial state of the mav to the trim state
    delta = trim_input  # set initial  input to constant constant trim input

    # openLoop()
    logdir_prefix = 'MPC01_'

    ## directory for logging
    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data')
    if not (os.path.exists(data_path)):
        os.makedirs(data_path)
    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)
    linearMPC(delta, trim_input, mav, logdir)
