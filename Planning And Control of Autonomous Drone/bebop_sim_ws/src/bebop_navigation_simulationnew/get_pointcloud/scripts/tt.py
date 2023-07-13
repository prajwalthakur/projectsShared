import numpy as np
z = 1.0
radius = 0.5
import pdb
center_x = [1,2,4]
center_y = [1,5,6]
list_circular_point= []
points = np.arange(0,2*np.pi,np.pi/180)
x_points = center_x + radius*np.cos(points)
pdb.set_trace()
x_points = x_points.reshape(x_points.shape[0],1)
y_points = center_y + radius*np.sin(points)
y_points = y_points.reshape(y_points.shape[0],1)
z_points = z*np.ones(x_points.shape[0])
z_points = z_points.reshape(z_points.shape[0],1)
points = np.hstack((x_points,y_points,z_points))
 
