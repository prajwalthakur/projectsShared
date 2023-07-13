"""
example of drawing a box-like spacecraft in python
    - Beard & McLain, PUP, 2012
    - Update history:
        1/8/2019 - RWB
"""
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
import pdb
class mav_viewer():
    def __init__(self,car,track,scale=5):
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Car with Track Viewer')
        self.window.setGeometry(0, 0, 1000, 1000)  # args: upper_left_x, upper_right_y, width, height
        # self.window.showMaximized()
        Gridsize = pg.QtGui.QVector3D(0.2,0.2,1)
        grid = gl.GLGridItem(size=Gridsize) # make a grid to represent the ground
        grid.scale(scale, scale, 1)
        grid.setColor('k')
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=200) # distance from center of plot to camera
        self.window.setBackgroundColor('w')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the spacecraft been plotted yet?
        self._named_items ={}
        # get points that define the non-rotated, non-translated spacecraft and the mesh colors
        self.car = car
        self.track = track
        self.points = self.car.get_car_vertices()
        self._addTrack()
        self.update(scale_Car=scale)
        #pdb.set_trace()

    ###################################
    def _addTrack(self):
        waypoints = self.track.dict_waypoints
        waypoints_inner,waypoints_center,waypoints_outer = self.track.get_coordinates(waypoints)                 
        range_waypoints = len(waypoints_center)-1
        self.lines("center_spline",np.asarray([waypoints_center[0:range_waypoints,0], waypoints_center[0:range_waypoints,1],np.zeros((waypoints_center[0:range_waypoints,0].shape))]).T,(1.0, 0.0 ,0.0,1.0))
        self.lines("outer_spline",np.asarray([waypoints_outer[0:range_waypoints,0], waypoints_outer[0:range_waypoints,1],np.zeros((waypoints_outer[0:range_waypoints,0].shape))]).T,(0.0, 1.0 ,0.0,1.0))
        self.lines("inner_spline",np.asarray([waypoints_inner[0:range_waypoints,0], waypoints_inner[0:range_waypoints,1],np.zeros((waypoints_inner[0:range_waypoints,0].shape))]).T,(0.0, 0.0 ,1.0,1.0))      
        return 
    def lines(self, name, lines, colors, alphas=1.0, width=1.0,
              antialias=True):
        if lines is None:
            return
        if name not in self._named_items:
            w_gl_item = gl.GLLinePlotItem(
                pos=5*lines,
                color=colors,
                width=width,
                antialias=antialias,
                mode='lines')
            self._named_items[name] = w_gl_item
            self.window.addItem(w_gl_item)
    
    # public functions
    def update(self,scale_Car=5):
        
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        car_points = self.car.get_car_vertices()
        mesh, meshColors = self._get_mesh_points(car_points)
        #mesh = self._points_to_mesh(car_points)
        # initialize the drawing the first time update() is called
        #vertices = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],[0, 0, 0],[0, 1, 0],[1, 1, 0]])  # Replace with your array
        #pdb.set_trace()
        if not self.plot_initialized:
            # initialize drawing of triangular mesh.
            self.body = gl.GLMeshItem(vertexes=scale_Car*mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=meshColors, # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
            self.window.addItem(self.body)  # add body to plot
            self.plot_initialized = True
        # vertices2 = np.array([[0, 0, 0],[0, 1, 0],[1, 1, 0]])  # Replace with your array

        # # initialize drawing of triangular mesh.
        # self.body2 = gl.GLMeshItem(vertexes=vertices2,  # defines the triangular mesh (Nx3x3)
        #                             vertexColors=meshColors, # defines mesh colors (Nx1)
        #                             drawEdges=True,  # draw edges between mesh elements
        #                             smooth=False,  # speeds up rendering
        #                             computeNormals=False)  # speeds up rendering
        #self.window.addItem(self.body2)  # add body to plot
        # vertices = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])  # Replace with your array

        # mesh = gl.GLMeshItem(vertexes=vertices, smooth=False, drawEdges=True)
        # else update drawing on all other calls to update()
        else:
            # reset mesh using rotated and translated points
            self.body.setMeshData(vertexes=scale_Car*mesh, vertexColors=meshColors)

        # update the center of the camera view to the spacecraft location
        #view_location = Vector(self.car.temporalState.x,self.car.temporalState.y,0)  # defined in ENU coordinates
        #self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()

    ###################################

    def _get_mesh_points(self,car_vertices):
        """"
            Points that define the spacecraft, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """


        points = car_vertices


        # scale points for better rendering
        scale = 1
        points = scale * points

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((2, 3, 4), dtype=np.float32)
        meshColors[0] = green
        meshColors[1] = green
        mesh = self._points_to_mesh(points)
        return mesh, meshColors

    def _points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        #pdb.set_trace()
        points = points.T
        points = np.hstack((points,np.zeros((4,1))))
        mesh= np.array([[points[0],
        points[1],
        points[2]],
       [points[2],
        points[3],
        points[0]]])
        return mesh

    def _Euler2Rotation(self, phi, theta, psi):
        """
        Converts euler angles to rotation matrix (R_b^i, i.e., body to inertial)
        """
        # only call sin and cos once for each angle to speed up rendering
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        R_roll = np.array([[1, 0, 0],
                           [0, c_phi, s_phi],
                           [0, -s_phi, c_phi]])
        R_pitch = np.array([[c_theta, 0, -s_theta],
                            [0, 1, 0],
                            [s_theta, 0, c_theta]])
        R_yaw = np.array([[c_psi, s_psi, 0],
                          [-s_psi, c_psi, 0],
                          [0, 0, 1]])
        R = R_roll @ R_pitch @ R_yaw  # inertial to body (Equation 2.4 in book)
        return R.T  # transpose to return body to inertial
