import numpy as np
from abc import ABC, abstractclassmethod
import matplotlib.pyplot as plt
import matplotlib.patches as plt_patches
import math
import scipy.integrate
from abc import abstractmethod
import pdb
# Temporal State Vector #
#########################

class TemporalState:
    def __init__(self, x, y, psi,v_x,v_y,w,virtual_control):
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
        self.v_x = v_x
        self.v_y = v_y
        self.w = w
        self.virtual_control = virtual_control

        self.members = ['x', 'y', 'psi','v_x','v_y','w','virtual_control']
    def __iadd__(self, other):
        """
        Overload Sum-Add operator.
        :param other: numpy array to be added to state vector
        """
        for state_id in range(len(self.members)):
            vars(self)[self.members[state_id]] += other[state_id]
        return self
class car_physical_model(ABC):
    """
    abstract class for car physical model , ( i.e will mention physical properties of car)

    modelParameters.sx = 7; %no of states
    modelParameters.su = 3; %number of inputs
    modelParameters.stateindex_x =1; % index of state: position x
    modelParameters.stateindex_y = 2; % index of state: position y
    modelParameters.stateindex_phi =3; % orientation/ heading angle
    modelParameters.stateindex_vx = 4; % longtitudinal velocity wrt body frame
    modelParameters.stateindex_vy = 5; %lateral velocity wrt body frame
    modelParameters.stateindex_omega =6; % yaw rate
    modelParameters.stateindex_theta = 7; % virtual position

    modelParameters.inputindex_DutyCycle = 1; % duty cycle 
    modelParameters.inputindex_delta = 2 ;  % steering angle
    modelParameters.inputindex_vtheta = 3 ; % virtual sped


    %% Paramaeters Based on Vechile 
    modelParameters.m = 0.041;
    modelParameters.Iz = 27.86e-6;
    modelParameters.lf = 0.029; % distance between com to front wheel
    modelParameters.lr = 0.033; % distance between com to real wheel

    modelParameters.Cm1 = 0.287;  % tire constant Based on Pacejka Tire Model
    modelParameters.Cm2 = 0.0545; % tire constant Based on Pacejka Tire Model
    modelParameters.Cr0=0.0518;
    modelParameters.Cr2=0.00035;

    modelParameters.Br = 3.3852;
    modelParameters.Cr = 1.2691;
    modelParameters.Dr = 0.1737;

    modelParameters.Bf = 2.579;
    modelParameters.Cf = 1.2;
    modelParameters.Df = 0.192;

    modelParameters.L = 0.12;
    modelParameters.W = 0.06;
    """
    def __init__(self):
        self.model_parameters ={}
        self.model_parameters["sx"] = 7 ; "no of state"
        self.model_parameters["su"]  = 3 
        self.model_parameters["stateindex_x"] = 0
        self.model_parameters["stateindex_y"] = 1 
        self.model_parameters["stateindex_phi"]  = 2 
        self.model_parameters["stateindex_vx"] = 3   
        self.model_parameters["stateindex_vy"] = 4
        self.model_parameters["stateindex_omega"] = 5   
        self.model_parameters["stateindex_theta"] = 6  
         
        self.model_parameters["inputindex_DutyCycle"] = 0 
        self.model_parameters["inputindex_delta"] = 1 
        self.model_parameters["inputindex_vthets"] = 2
        
        
        
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
    def __init__(self, Ts):
        """
        :param reference_path : reference path object to follow
        : param length length of car in m        
        """
        super(bicycle_dynamic_model, self).__init__()
        self.deltaT = self.model_parameters["dt"];   " dynamical model timestep "
        self.Ts = Ts ;          " dt is the simulation time step "
        
    
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
        self.temporalState = TemporalState(solution.y[0],solution.y[1],solution.y[2],solution.y[3],solution.y[4],solution.y[5],solution.y[6])
        return solution
            
    def get_next_state_complex(self,t,x,u):
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
        omega = x[self.model_parameters["stateindex_omega"]]
        v_x = x[self.model_parameters["stateindex_vx"]]
        v_y = x[self.model_parameters["stateindex_vy"]]

        delta = u[self.model_parameters["inputindex_delta"]]
        d= u[self.model_parameters["inputindex_DutyCycle"]]

        m = self.model_parameters["m"]
        Iz = self.model_parameters["Iz"]
        l_f = self.model_parameters["lf"]
        l_r = self.model_parameters["lr"]
        
        alpha_f = -np.arctan2(omega*l_f + v_y,abs(v_x)) + delta    
        alpha_r = np.arctan2(omega*l_r-v_y,abs(v_x))
        F_fy = D_f*np.sin(C_f*np.arctan(B_f*alpha_f))
        F_ry = D_r*np.sin(C_r*np.arctan(B_r*alpha_r))
        F_rx = Cm1*d -Cm2*v_x*d - Cro-Cr2*(v_x**2)
        "xdot = X_dot , Y_dot , phi_dot , v_x_dot,v_y_dot , w_dot ,u(3)(check needed)"
        xdot = np.asarray([[v_x*np.cos(phi) - v_y*np.sin(phi)],
                        [v_y*np.cos(phi) + v_x*np.sin(phi)] ,
                        [omega],
                        [(F_rx-F_fy*np.sin(delta)+ m*v_y*omega)/m],
                        [(F_ry + F_fy*np.cos(delta)-m*v_x*omega)/m],
                        [(F_fy*l_f*np.cos(delta)-F_ry*l_r)/Iz],
                        [u[2]]])
        
        #Simple Bicycle Model
        # xdot = np.asarray([[v_x*np.cos(phi) ],
        #         [v_x*np.sin(phi)] ,
        #         [v_x*np.tan(delta)/l_f],
        #         [u[2]]])
        
        return xdot.squeeze()
    def get_next_state(self,t,x,u):
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
        omega = x[self.model_parameters["stateindex_omega"]]
        v_x = x[self.model_parameters["stateindex_vx"]]
        v_y = x[self.model_parameters["stateindex_vy"]]

        delta = u[self.model_parameters["inputindex_delta"]]
        d= u[self.model_parameters["inputindex_DutyCycle"]]

        m = self.model_parameters["m"]
        Iz = self.model_parameters["Iz"]
        l_f = self.model_parameters["lf"]
        l_r = self.model_parameters["lr"]
        
        alpha_f = -np.arctan2(omega*l_f + v_y,abs(v_x)) + delta    
        alpha_r = np.arctan2(omega*l_r-v_y,abs(v_x))
        F_fy = D_f*np.sin(C_f*np.arctan(B_f*alpha_f))
        F_ry = D_r*np.sin(C_r*np.arctan(B_r*alpha_r))
        F_rx = Cm1*d -Cm2*v_x*d - Cro-Cr2*(v_x**2)
        "xdot = X_dot , Y_dot , phi_dot , v_x_dot,v_y_dot , w_dot ,u(3)(check needed)"
        # xdot = np.asarray([[v_x*np.cos(phi) - v_y*np.sin(phi)],
        #                 [v_y*np.cos(phi) + v_x*np.sin(phi)] ,
        #                 [omega],
        #                 [(F_rx-F_fy*np.sin(delta)+ m*v_y*omega)/m],
        #                 [(F_ry + F_fy*np.cos(delta)-m*v_x*omega)/m],
        #                 [(F_fy*l_f*np.cos(delta)-F_ry*l_r)/Iz],
        #                 [u[2]]])
        
        #Simple Bicycle Model
        xdot = np.asarray([[v_x*np.cos(phi) ],
                [v_x*np.sin(phi)] ,
                [v_x*np.tan(delta)/l_f],
                [u[2]]])
        
        return xdot.squeeze()

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
if __name__ == '__main__':
    car = bicycle_dynamic_model(0.02)
    x0 = np.asarray([-0.8457,1.0979,-0.7854,1.0000,0,0,0])
    #x0 = np.expand_dims(x0,axis=1)
    u_current = np.asarray([0,0,0])
    sol = car.get_simulation_next_state(x0,u_current)
    #plot_simple_Car(2,2,-np.pi/3)
        