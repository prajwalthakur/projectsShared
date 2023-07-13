import pdb

import matplotlib.pyplot as plt
import numpy as np
# from MPCC import MPCC
from scipy import sparse

from dynamic_bicycle_model import bicycle_dynamic_model
from map import Map, Obstacle
from reference_path_mpcc import ReferencePath
from car_viewer import mav_viewer
if __name__ == '__main__':
        reference_path = ReferencePath(0.025,
                                smoothing_distance=5, max_width=2.0,
                                circular=True)
       
        
       
        # Instantiate motion model
        x0 = np.asarray([-0.29,-1.59,-0.7854,1.0000])
        car = bicycle_dynamic_model(x0,0.02,reference_path) #Ts = 0.02
        mav = mav_viewer(car,reference_path)
        #x0 = np.asarray([-0.8457,1.0979,-0.7854,1.0000])
        #x0 = np.expand_dims(x0,axis=1)
        u_current = np.asarray([0,0,0])
        sol = car.get_simulation_next_state(x0,u_current) #initial state
        pdb.set_trace()
        #car.plot_simple_Car()
        #reference_path._plot_map()
        #plt.axis("on")
        #plt.pause(0.001)


        #mpcc = MPCC()
        # Until arrival at end of path
        while car.arc < reference_path.length:

                # Get control signals
                u_current = np.asarray([0.5,0.1,5])#mpcc.get_control()
                sol = car.get_simulation_next_state(x0,u_current) #initial state
                x0 = sol
                print(x0)
                print("x=",mav.car.temporalState.x,"y=",mav.car.temporalState.y)
                mav.update()
                #car.plot_simple_Car()
                #car.show()
                #plt.axis("on")
                #plt.pause(0.001)
                sec = input('Let us wait for user input')
