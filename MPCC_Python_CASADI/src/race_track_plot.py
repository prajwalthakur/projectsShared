from CubicSpline import Spline2D
import numpy as np
import math
from load_map import load_map2
import pdb
print("Spline 2D test")
import matplotlib.pyplot as plt
waypoints = load_map2()
# ds = 0.05  # [m] distance of each intepolated points
# x= []
# y =[]

# wr_right =[]
# wr_left =[]
# xy =[]
# for i in range(len(waypoints)-1):
#     x.append(waypoints[i][0])
#     y.append(waypoints[i][1])
#     xy.append((waypoints[i][0],waypoints[i][1]))
#     wr_right.append(waypoints[i][2])
#     wr_left.append(waypoints[i][3])
# n_wp = [(np.sqrt((x[i+1] - x[i])**2 + 
#                 (y[i+1] - x[i])**2)) for i in range(len(x)-1)]

# sp = Spline2D(x, y)
# s = np.arange(0, sp.s[-1], ds)
# #pdb.set_trace()
# rx, ry, ryaw, rk = [], [], [], []
# for i_s in s:
#     ix, iy = sp.calc_position(i_s)
#     rx.append(ix)
#     ry.append(iy)
#     ryaw.append(sp.calc_yaw(i_s))
#     rk.append(sp.calc_curvature(i_s))
# point_rights =np.empty((0,2))
# point_lefts = np.empty((0,2))
# for i_s in sp.s[0:-1]:
#     ix, iy = sp.calc_position(i_s)
#     tangent = sp.calc_yaw(i_s)
#     idx =xy.index((ix,iy))
#     wr = wr_right[idx]
#     wl = wr_left[idx]
#     point_rights = np.vstack((point_rights ,np.array([ix + wr*math.cos(tangent - math.pi ) , iy + wr*math.sin(tangent - math.pi) ])))
#     point_lefts = np.vstack((point_lefts ,np.array([ix + wl*math.cos(tangent + math.pi ) , iy + wl*math.sin(tangent + math.pi) ])))
# sp_bd_right = Spline2D(point_rights[0:,0], point_rights[0:,1])
# sp_bd_left = Spline2D(point_lefts[0:,0], point_lefts[0:,1])
# s_right = np.arange(0, sp_bd_right.s[-1], ds)
# s_left = np.arange(0, sp_bd_left.s[-1], ds)
# pdb.set_trace()
# rx_right, ry_right, rx_left, ry_left = [], [], [], []
# for i_s in range(len(s_left)-1):
#     ix_right, iy_right = sp.calc_position(s_right[i_s])
#     ix_left,iy_left = sp.calc_position(s_left[i_s])
#     rx_right.append(ix_right)
#     ry_right.append(iy_right)
#     rx_left.append(ix_left)
#     ry_left.append(iy_left) 

# flg, ax = plt.subplots(1)
# #pdb.set_trace()
# # plt.plot(x, y, "xb", label="input")
# plt.plot(rx, ry, "-r", label="spline")
# plt.plot(point_rights[0:,0], point_rights[0:,], "-g", label="bd_spline_right")
# plt.plot(point_lefts[0:,], point_lefts[:,], "-b", label="bd_spline_left")
# plt.grid(True)
# plt.axis("equal")
# plt.xlabel("x[m]")
# plt.ylabel("y[m]")
# plt.legend()
# plt.show()

##############################to load from matlab #######################

ds = 0.05  # [m] distance of each intepolated points
x= []
y =[]

wr_right =[]
wr_left =[]
xy =[]
for i in range(len(waypoints)-1):
    x.append(waypoints[i][0])
    y.append(waypoints[i][1])
    xy.append((waypoints[i][0],waypoints[i][1]))
    wr_right.append(waypoints[i][2])
    wr_left.append(waypoints[i][3])
n_wp = [(np.sqrt((x[i+1] - x[i])**2 + 
                (y[i+1] - x[i])**2)) for i in range(len(x)-1)]

sp = Spline2D(x, y)
s = np.arange(0, sp.s[-1], ds)
#pdb.set_trace()
rx, ry, ryaw, rk = [], [], [], []
for i_s in s:
    ix, iy = sp.calc_position(i_s)
    rx.append(ix)
    ry.append(iy)
    ryaw.append(sp.calc_yaw(i_s))
    rk.append(sp.calc_curvature(i_s))
point_rights =np.empty((0,2))
point_lefts = np.empty((0,2))
for i_s in sp.s[0:-1]:
    ix, iy = sp.calc_position(i_s)
    tangent = sp.calc_yaw(i_s)
    idx =xy.index((ix,iy))
    wr = wr_right[idx]
    wl = wr_left[idx]
    point_rights = np.vstack((point_rights ,np.array([ix + wr*math.cos(tangent - math.pi ) , iy + wr*math.sin(tangent - math.pi) ])))
    point_lefts = np.vstack((point_lefts ,np.array([ix + wl*math.cos(tangent + math.pi ) , iy + wl*math.sin(tangent + math.pi) ])))
sp_bd_right = Spline2D(point_rights[0:,0], point_rights[0:,1])
sp_bd_left = Spline2D(point_lefts[0:,0], point_lefts[0:,1])
s_right = np.arange(0, sp_bd_right.s[-1], ds)
s_left = np.arange(0, sp_bd_left.s[-1], ds)
pdb.set_trace()
rx_right, ry_right, rx_left, ry_left = [], [], [], []
for i_s in range(len(s_left)-1):
    ix_right, iy_right = sp.calc_position(s_right[i_s])
    ix_left,iy_left = sp.calc_position(s_left[i_s])
    rx_right.append(ix_right)
    ry_right.append(iy_right)
    rx_left.append(ix_left)
    ry_left.append(iy_left) 

flg, ax = plt.subplots(1)
#pdb.set_trace()
# plt.plot(x, y, "xb", label="input")
plt.plot(rx, ry, "-r", label="spline")
plt.plot(point_rights[0:,0], point_rights[0:,], "-g", label="bd_spline_right")
plt.plot(point_lefts[0:,], point_lefts[:,], "-b", label="bd_spline_left")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()
plt.show()