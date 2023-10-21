import pdb

import matplotlib.pyplot as plt
import numpy as np
# from MPCC import MPCC
from scipy import sparse

from dynamic_bicycle_model import bicycle_dynamic_model
from map import Map, Obstacle
from reference_path_mpcc import ReferencePath
from car_viewer import mav_viewer
import delayedMPC
def get_next_state(car,v_constant,tau_u,tau_d,u_current,control_seqs,x0):
        sol = car.get_simulation_next_state(x0,np.hstack((u_current,v_constant)),T=tau_u+tau_d) #initial state
        u_current = control_seqs[0]
        x0 = sol
        sol = car.get_simulation_next_state(x0,np.hstack((u_current,v_constant)), T=0.02 - (tau_u+tau_d)) #initial state
        x0 = sol
        return x0
def get_open_loop(car,v_constant,tau_u,tau_d,u_current,control_seqs,x0):
        openLoop_states = np.empty((1,3))
        for i in range(len(control_seqs)):
                openLoop_states =np.vstack((openLoop_states,get_next_state(car,v_constant,tau_u,tau_d,u_current,control_seqs[i:],x0)))
                x0=openLoop_states[-1]
        return openLoop_states
if __name__ == '__main__':
        reference_path = ReferencePath(0.025,
                                smoothing_distance=5, max_width=2.0,
                                circular=True)
       
        
        v_constant = 0.3
        # Instantiate motion model
        x0 = np.asarray([-0.29,-1.59,+0.7854])
        car = bicycle_dynamic_model(x0,0.02,reference_path) #Ts = 0.02
        mav = mav_viewer(car,reference_path)
        #x0 = np.asarray([-0.8457,1.0979,-0.7854,1.0000])
        #x0 = np.expand_dims(x0,axis=1)
        u_current = np.asarray([0])
        sol = car.get_simulation_next_state(x0,np.hstack((u_current,v_constant)),T=0.02) #initial state
        #car.plot_simple_Car()
        #reference_path._plot_map()
        #plt.axis("on")
        #plt.pause(0.001)


        delayed_mpc = delayedMPC.delayed_mpc(ts_control=0.02)
        # Until arrival at end of path
        x_ref = 0.0
        y_ref = 0.0
        psi_ref = 0.0
        tau_u = 0.03
        tau_d = 0.01
        l_f = car.model_parameters["lf"]
        weights_mpc = np.asarray([20,20,1,1]) #first three for states , last one for input 
        #pdb.set_trace()
        while car.arc < reference_path.length:
                # Get control signals
                control_seqs,_ = delayed_mpc.update(x_ref, y_ref , psi_ref, state_current=x0, input_last=u_current, weights=weights_mpc,v_cosnt=v_constant,l_f=l_f,tau_u=tau_u,tau_d=tau_d)
                x_open_loop_predictions = get_open_loop(car,v_constant,tau_u,tau_d,u_current,control_seqs,x0)
                #pdb.set_trace()
                sol = get_next_state(car,v_constant,tau_u,tau_d,u_current,control_seqs,x0)
                x0=sol
                u_current = control_seqs[0]
                print(x0)
                print("x=",mav.car.temporalState.x,"y=",mav.car.temporalState.y)
                print("open_loop_prediction_shape=",x_open_loop_predictions.shape)
                mav.update(x_open_loop_predictions)
                #car.plot_simple_Car()
                #car.show()
                #plt.axis("on")
                #plt.pause(0.001)
                sec = input('Let us wait for user input')
               