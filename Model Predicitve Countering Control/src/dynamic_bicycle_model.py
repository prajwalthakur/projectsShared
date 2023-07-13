import numpy as np
from abc import ABC, abstractclassmethod
import matplotlib.pyplot as plt
import matplotlib.patches as plt_patches
import math
import scipy.integrate
from abc import abstractmethod
import pdb
import findtheta
# Temporal State Vector #
#########################
# Colors
CAR = '#F1C40F'
CAR_OUTLINE = '#B7950B'
class TemporalState:
    def __init__(self, x, y, psi,virtual_control):
        """
        Temporal State Vector containing car pose (x, y, psi)
        :param x: x position in global coordinate system | [m]
        :param y: y position in global coordinate system | [m]
        :param psi: yaw angle | [rad]
        """
        #xdot = X_dot , Y_dot , phi_dot , v_x_dot,v_y_dot , w_dot ,u(3)
        self.x = x
        self.y = y
        self.psi = psi
        self.virtual_control = virtual_control

        self.members = ['x', 'y', 'psi','virtual_control']
    def __iadd__(self, other):
        """
        Overload Sum-Add operator.
        :param other: numpy array to be added to state vector
        """
        for state_id in range(len(self.members)):
            vars(self)[self.members[state_id]] += other[state_id]
        return self
class car_physical_model(ABC):
    def __init__(self):
        self.model_parameters ={}
        self.model_parameters["sx"] = 3 ; "no of state"
        self.model_parameters["su"]  = 3 
        self.model_parameters["stateindex_x"] = 0
        self.model_parameters["stateindex_y"] = 1 
        self.model_parameters["stateindex_phi"]  = 2 
        #self.model_parameters["stateindex_v"] = 3   
        #self.model_parameters["stateindex_vy"] = 4
        #self.model_parameters["stateindex_omega"] = 5   
        #self.model_parameters["stateindex_theta"] = 6  
         
        #self.model_parameters["inputindex_DutyCycle"] = 0 
        self.model_parameters["inputindex_delta"] = 0 
        self.model_parameters["inputindex_vthets"] = 1
        
        
        
        self.model_parameters["m"] = 0.041 
        self.model_parameters["Iz"] = 27.86e-6
        self.model_parameters["lf"] = 0.029
        self.model_parameters["lr"] = 0.033
        
        self.model_parameters["Cm1"] = 0.287
        self.model_parameters["Cm2"] = 0.0545
         
        self.model_parameters["Cro"] = 0.0518
        self.model_parameters["Cr2"] = 0.00035

        self.model_parameters["Br"] = 3.3852
        self.model_parameters["Cr"] = 1.2691
        self.model_parameters["Dr"] = 0.1737

        self.model_parameters["Bf"] = 2.579
        self.model_parameters["Cf"] = 1.2
        self.model_parameters["Df"] = 0.192
        
        self.model_parameters["L"] = 0.12
        self.model_parameters["W"] = 0.06
        self.model_parameters["dt"] = 0.002
                
        self.model_parameters["Scale"] = 1  
        
class bicycle_dynamic_model(car_physical_model):
    def __init__(self, x_init,Ts,reference_path):
        """
        :param reference_path : reference path object to follow
        : param length length of car in m        
        """
        super(bicycle_dynamic_model, self).__init__()
        self.deltaT = self.model_parameters["dt"];   " dynamical model timestep "
        self.Ts = Ts ;          " dt is the simulation time step "
        self.reference_path = reference_path
        _,self.wayPointCenter,_ = self.reference_path.get_coordinates(self.reference_path.dict_waypoints)
        self.Trackwidth = self.reference_path.Trackwidth
        _,self.theta_coordinates,_ = self.reference_path.get_theta_coordinates()
        self.s = None
        self.closestIndex = None
        self.last_closestIdx = 0
        self.temporalState = TemporalState(x_init[0],x_init[1],x_init[2],x_init[3])
    
    def get_simulation_next_state(self,x_current,u_current):
        i=0
        x_0 = x_current
        # while( (i+1) * self.Ts <= self.deltaT):
        #     x_next = self.get_next_state(x_current,u_current)
        #     x_current = x_next
        #     i+=1
        # return x_current
        solution= scipy.integrate.solve_ivp(self.get_next_state , t_span=[0,self.Ts] ,y0 =x_0,t_eval=[self.Ts],args=[u_current])
        v = solution.t
        s = solution.y
        #pdb.set_trace()
        self.temporalState = TemporalState(solution.y[0],solution.y[1],solution.y[2],solution.y[3])
        self.arc,self.closestIndex = findtheta.findTheta(currentPose=np.asarray([[self.temporalState.x,self.temporalState.y]]),TrackCenter=self.wayPointCenter,theta_coordinates=self.theta_coordinates,trackWidth=self.Trackwidth, last_closestIdx=self.last_closestIdx)
        self.last_closestIdx = self.closestIndex
        return np.squeeze(np.asarray([solution.y[0],solution.y[1],solution.y[2],solution.y[3]]))

    def get_next_state(self,t,x,u,v=0.1):
        x_next     = np.zeros((self.model_parameters["sx"],1))
        cur_x_next = np.zeros((self.model_parameters["sx"],1))
        Cm1 = self.model_parameters["Cm1"]
        Cm2 = self.model_parameters["Cm2"]
        Cro = self.model_parameters["Cro"]
        Cr2 = self.model_parameters["Cr2"]
        B_f = self.model_parameters["Bf"]
        C_f = self.model_parameters["Cf"]
        D_f = self.model_parameters["Df"]

        B_r = self.model_parameters["Br"]
        C_r = self.model_parameters["Cr"]
        D_r = self.model_parameters["Dr"]
        phi = x[self.model_parameters["stateindex_phi"]]
        #omega = x[self.model_parameters["stateindex_omega"]]
        #v_x = x[self.model_parameters["stateindex_vx"]]
        #v_y = x[self.model_parameters["stateindex_vy"]]

        delta = u[self.model_parameters["inputindex_delta"]]
        #d= u[self.model_parameters["inputindex_DutyCycle"]]

        m = self.model_parameters["m"]
        Iz = self.model_parameters["Iz"]
        l_f = self.model_parameters["lf"]
        l_r = self.model_parameters["lr"]
        
        # alpha_f = -np.arctan2(omega*l_f + v_y,abs(v_x)) + delta    
        # alpha_r = np.arctan2(omega*l_r-v_y,abs(v_x))
        # F_fy = D_f*np.sin(C_f*np.arctan(B_f*alpha_f))
        # F_ry = D_r*np.sin(C_r*np.arctan(B_r*alpha_r))
        # F_rx = Cm1*d -Cm2*v_x*d - Cro-Cr2*(v_x**2)
        "xdot = X_dot , Y_dot , phi_dot , v_x_dot,v_y_dot , w_dot ,u(3)(check needed)"
        # xdot = np.asarray([[v_x*np.cos(phi) - v_y*np.sin(phi)],
        #                 [v_y*np.cos(phi) + v_x*np.sin(phi)] ,
        #                 [omega],
        #                 [(F_rx-F_fy*np.sin(delta)+ m*v_y*omega)/m],
        #                 [(F_ry + F_fy*np.cos(delta)-m*v_x*omega)/m],
        #                 [(F_fy*l_f*np.cos(delta)-F_ry*l_r)/Iz],
        #                 [u[2]]])
        
        #Simple Bicycle Model
        v = u[0]
        delta =u[1]
        xdot = np.asarray([[v*np.cos(phi) ],
                [v*np.sin(phi)] ,
                [v*np.tan(delta)/l_f],
                [u[2]]])
        
        return xdot.squeeze()

    def get_car_vertices(self):
        l = self.model_parameters["L"]  # length of mobile robot
        w = self.model_parameters["W"]   # width of mobile robot
        x = self.temporalState.x
        y = self.temporalState.y
        psi = self.temporalState.psi
        # Mobile robot coordinates wrt body frame
        #pdb.set_trace()
        mr_co = np.array([[-l/2, l/2, l/2, -l/2],
                        [-w/2, -w/2, w/2, w/2]])
        R_psi = np.array([[np.cos(psi), -np.sin(psi)],
                            [np.sin(psi), np.cos(psi)]])  # rotation matrix
        v_pos = np.dot(np.squeeze(R_psi), np.squeeze(mr_co))  # orientation w.r.t intertial frame              
        v_pos[0,:] = v_pos[0, :] + x
        v_pos[1,:] = v_pos[1, :] + y
        return v_pos
    def plot_simple_Car(self):
        l = self.model_parameters["L"]  # length of mobile robot
        w = self.model_parameters["W"]   # width of mobile robot
        x = self.temporalState.x[0]
        y = self.temporalState.y[0]
        psi = self.temporalState.psi[0]
        # Mobile robot coordinates wrt body frame
        #pdb.set_trace()
        mr_co = np.array([[-l/2, l/2, l/2, -l/2],
                        [-w/2, -w/2, w/2, w/2]])
        R_psi = np.array([[np.cos(psi), -np.sin(psi)],
                            [np.sin(psi), np.cos(psi)]])  # rotation matrix
        v_pos = np.dot(R_psi, mr_co)  # orientation w.r.t intertial frame
        ax =plt.gca()
        ax.fill(v_pos[0, :] + x, v_pos[1, :] + y, 'g')  # rotation + translation to get global coordinates  
        #plt.legend(['MR'], fontsize=24)
        # plt.xlabel('x,[m]')
        # plt.ylabel('y,[m]')
        # plt.axis([-1, 3, -1, 3])
        # plt.axis('square')
        # plt.grid(True)
        # plt.show(block=False)
        # plt.pause(0.1)
        # plt.clf()  
        return 
    def show(self):
        """
        Display car on current axis.
        """
        l = self.model_parameters["L"]  # length of mobile robot
        w = self.model_parameters["W"]   # width of mobile robot
        x = self.temporalState.x[0]
        y = self.temporalState.y[0]
        psi = self.temporalState.psi[0]
        # Get car's center of gravity
        cog = (x, y)
        # Get current angle with respect to x-axis
        yaw = np.rad2deg(psi)
        # Draw rectangle
        car = plt_patches.Rectangle(cog, width=w, height=w,
                                    angle=yaw, facecolor=CAR,
                                    edgecolor=CAR_OUTLINE, zorder=20)

        # Shift center rectangle to match center of the car
        car.set_x(car.get_x() - (l / 2 *
                                 np.cos(psi) -
                                 w / 2 *
                                 np.sin(psi)))
        car.set_y(car.get_y() - (w / 2 *
                                 np.cos(psi) +
                                 l / 2 *
                                 np.sin(psi)))

        # Add rectangle to current axis
        ax = plt.gca()
        ax.add_patch(car)
if __name__ == '__main__':
    car = bicycle_dynamic_model(0.02)
    x0 = np.asarray([-0.8457,1.0979,-0.7854,1.0000,0,0,0])
    #x0 = np.expand_dims(x0,axis=1)
    u_current = np.asarray([0,0,0])
    sol = car.get_simulation_next_state(x0,u_current)
    #plot_simple_Car(2,2,-np.pi/3)
        