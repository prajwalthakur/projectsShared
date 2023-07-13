import numpy as np
import math
from map import Map, Obstacle
from skimage.draw import line_aa
import matplotlib.pyplot as plt
from scipy import sparse
import CubicSpline
import osqp
import pdb
from load_map import load_map2
import border_adjustment
from dynamic_bicycle_model import bicycle_dynamic_model
# Colors
DRIVABLE_AREA = '#BDC3C7'
WAYPOINTS = '#D0D3D4'
PATH_CONSTRAINTS = '#F5B041'
OBSTACLE = '#2E4053'


############
# Waypoint #
############

class Waypoint:
    def __init__(self, x, y, psi, kappa,dist):
        """
        Waypoint object containing x, y location in global coordinate system,
        orientation of waypoint psi and local curvature kappa. Waypoint further
        contains an associated reference velocity computed by the speed profile
        and a path width specified by upper and lower bounds.
        :param x: x position in global coordinate system | [m]
        :param y: y position in global coordinate system | [m]
        :param psi: orientation of waypoint | [rad]
        :param kappa: local curvature | [1 / m]
        """
        self.x = x
        self.y = y
        self.psi = psi
        self.kappa = kappa
        self.dist = dist
        # Reference velocity at this waypoint according to speed profile
        self.v_ref = None

        # Information about drivable area at waypoint
        # upper and lower bound of drivable area orthogonal to
        # waypoint orientation.
        # Upper bound: free drivable area to the left of center-line in m
        # Lower bound: free drivable area to the right of center-line in m
        self.lb = None
        self.ub = None
        self.static_border_cells = None
        self.dynamic_border_cells = None

    # def __sub__(self, other):
    #     """
    #     Overload subtract operator. Difference of two waypoints is equal to
    #     their euclidean distance.
    #     :param other: subtrahend
    #     :return: euclidean distance between two waypoints
    #     """
    #     return ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5


##################
# Reference Path #
##################


class ReferencePath:
    def __init__(self, resolution, smoothing_distance,
                 max_width, circular):
        """
        Reference Path object. Create a reference trajectory from specified
        corner points with given resolution. Smoothing around corners can be
        applied. Waypoints represent center-line of the path with specified
        maximum width to both sides.
        :param map: map object on which path will be placed
        :param wp_x: x coordinates of corner points in global coordinates
        :param wp_y: y coordinates of corner points in global coordinates
        :param resolution: resolution of the path in m/wp
        :param smoothing_distance: number of waypoints used for smoothing the
        path by averaging neighborhood of waypoints
        :param max_width: maximum width of path to both sides in m
        :param circular: True if path circular
        """

        # Precision
        self.eps = 1e-12

        # Map
        self.map = map

        # Resolution of the path
        self.resolution = resolution

        # Look ahead distance for path averaging
        self.smoothing_distance = smoothing_distance

        # Circular flag
        self.circular = circular

        # List of waypoint objects
        self.waypoints_list = self._create_map()
        # Number of waypoints
        self.dict_waypoints = {"waypoints_inner": self.waypoints_list[0:,0] , "waypoints_center" : self.waypoints_list[0:,1], "waypoints_outer":self.waypoints_list[0:,2]}
        #self._plot_map()
        self.n_waypoints = len(self.waypoints_list)
        # Length of path
        self.length= self._compute_length()

        # Compute path width (attribute of each waypoint)
        #self._compute_width(max_width=max_width)
        #pdb.set_trace()
        self.Trackwidth =  ((self.dict_waypoints["waypoints_inner"][0].x- self.dict_waypoints["waypoints_outer"][0].x)**2 + (self.dict_waypoints["waypoints_inner"][0].y- self.dict_waypoints["waypoints_outer"][0].y)**2)**0.5
       
        
    def _construct_path(self, wp_x, wp_y,no_points=None):
        # no of interpolated-waypoints considering resolution
        n_wp = [int(np.sqrt((wp_x[i+1] - wp_x[i])**2 + 
                    (wp_y[i+1] - wp_x[i])**2)/self.resolution) for i in range(len(wp_x)-1)]
        
        #splinfy using cubic polynomial-spline (mpcc paper implemented there own spline function , method not mentioned in the paper)
        
        spline_object = CubicSpline.Spline2D(wp_x,wp_y)
        if (no_points == None):
            resolution = self.resolution
        else :
            resolution = spline_object.s[-1]/no_points
        sp_new_wp_dist = np.arange(0,spline_object.s[-1],resolution)
        gp_x, gp_y = wp_x[-1], wp_y[-1]
        waypoints =np.empty((0,1))
        rx, ry, ryaw, rk = [], [], [], []
        for wp_dist in sp_new_wp_dist :
            ix,iy = spline_object.calc_position(wp_dist)
            ryaw = spline_object.calc_yaw(wp_dist)
            rk = spline_object.calc_curvature(wp_dist)
            rx.append(ix)
            ry.append(iy)
            #pdb.set_trace()
            wp = np.asarray([Waypoint(x=ix,y=iy,psi=ryaw,kappa=rk,dist=wp_dist)])
            waypoints = np.vstack((waypoints,wp))
        return waypoints
    
    def _compute_length(self):
        """
        Compute length of center-line path as sum of euclidean distance between
        waypoints.
        :return: length of center-line path in m
        """
        _,waypoints,_ = self.get_coordinates(self.dict_waypoints)
        segment_lengths = [0.0] + [np.linalg.norm(waypoints[wp_id+1] - waypoints
                    [wp_id]) for wp_id in range(len(waypoints)-1)]
        s = sum(segment_lengths)
        return s
    
    def _create_map(self):
        old_waypoints = load_map2()
        ModelParams = {"W":0.06,"Scale":1}
        safetyScaling = 1.5
        waypoints,_ = border_adjustment.border_adjustment(old_waypoints,ModelParams,safetyScaling)

        
        #waypoints = np.vstack((waypoints)) # to join some gaps left while creating the map
        waypoints_center = self._construct_path(waypoints[0:,2],waypoints[0:,3])
        l=len(waypoints_center)
        waypoints_outer = self._construct_path(waypoints[0:,4],waypoints[0:,5],no_points=l)
        waypoints_inner = self._construct_path(waypoints[0:,0],waypoints[0:,1],no_points=l)  
        waypoints_list  = np.hstack((waypoints_inner,waypoints_center,waypoints_outer))
        return waypoints_list
        #pdb.set_trace()
    
    def get_coordinates(self,waypoints):
        wp_center = waypoints["waypoints_center"]
        wp_inner = waypoints["waypoints_inner"]
        wp_outer = waypoints["waypoints_outer"]
        xy_inner = np.empty((0,2))
        xy_center = np.empty((0,2))
        xy_outer = np.empty((0,2))
        #pdb.set_trace()
        for i,c,outer in zip(wp_inner,wp_center,wp_outer):
            xy_inner = np.vstack((xy_inner,np.array([i.x,i.y])))
            xy_center = np.vstack((xy_center,np.array([c.x,c.y])))
            xy_outer = np.vstack((xy_outer,np.array([outer.x,outer.y])))
        return xy_inner,xy_center,xy_outer
    
    def get_theta_coordinates(self):
        wp_center = self.dict_waypoints["waypoints_center"]
        wp_inner = self.dict_waypoints["waypoints_inner"]
        wp_outer = self.dict_waypoints["waypoints_outer"]
        xy_inner = np.empty((0,4))
        xy_center = np.empty((0,4))
        xy_outer = np.empty((0,4))
        #pdb.set_trace()
        idx=0
        for i,c,outer in zip(wp_inner,wp_center,wp_outer):
            xy_inner = np.vstack((xy_inner,np.array([idx,i.x,i.y,i.dist])))
            xy_center = np.vstack((xy_center,np.array([idx,c.x,c.y,c.dist])))
            xy_outer = np.vstack((xy_outer,np.array([idx,outer.x,outer.y,outer.dist])))
            idx+=1
        return xy_inner,xy_center,xy_outer
    
    def _plot_map(self):
        waypoints = self.dict_waypoints
        waypoints_inner,waypoints_center,waypoints_outer = self.get_coordinates(waypoints)                 
        flg,ax = plt.subplots(1)
        range_waypoints = len(waypoints_center)-1
        plt.plot(waypoints_center[0:range_waypoints,0], waypoints_center[0:range_waypoints,1], "-r", label="center_spline")
        plt.plot(waypoints_outer[0:range_waypoints,0], waypoints_outer[0:range_waypoints,1], "-g", label="outer_spline")
        plt.plot(waypoints_inner[0:range_waypoints,0], waypoints_inner[0:range_waypoints,1], "-b", label="inner_spline")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        #pdb.set_trace()
        #test
        #self.plot_simple_Car(waypoints_center[0,0],waypoints_center[0,1],0)
        #plt.show() 
        return
    def plot_simple_Car(self,x,y,psi):
        l = 0.12 #self.model_parameters["L"]  # length of mobile robot
        w = 0.06 #self.model_parameters["W"]   # width of mobile robot

        # Mobile robot coordinates wrt body frame
        mr_co = np.array([[-l/2, l/2, l/2, -l/2],
                        [-w/2, -w/2, w/2, w/2]])
        R_psi = np.array([[np.cos(psi), -np.sin(psi)],
                            [np.sin(psi), np.cos(psi)]])  # rotation matrix
        v_pos = np.dot(R_psi, mr_co)  # orientation w.r.t intertial frame
        plt.fill(v_pos[0, :] + x, v_pos[1, :] + y, 'g')  # rotation + translation to get global coordinates
        # plt.legend(['MR'], fontsize=24)
        # plt.xlabel('x,[m]')
        # plt.ylabel('y,[m]')
        # plt.axis([-1, 3, -1, 3])
        # plt.axis('square')
        # plt.grid(True)
        # plt.show(block=False)
        # pdb.set_trace()
        # plt.pause(0.1)
        # plt.clf()    
        return 
    
if __name__ == '__main__':
    reference_path = ReferencePath(0.025,
                                smoothing_distance=5, max_width=2.0,
                                circular=True)
    reference_path._plot_map()
    car = bicycle_dynamic_model(0.02)
    #x0 = np.asarray([-0.8457,1.0979,-0.7854,1.0000,0,0,0])
    x0 = np.asarray([-0.8457,1.0979,-0.7854,1.0000])
    #x0 = np.expand_dims(x0,axis=1)
    u_current = np.asarray([0,0,0])
    sol = car.get_simulation_next_state(x0,u_current)
    car.plot_simple_Car()
    plt.axis("on")
    plt.pause(0.001)
    pdb.set_trace()