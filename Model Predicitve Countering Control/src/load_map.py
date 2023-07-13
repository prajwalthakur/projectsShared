import numpy as np
import csv
import pdb
import scipy.io
import pandas as pd
import matplotlib.pyplot as plt
def load_map():
    file = open("racetrack-database-master/tracks/Austin.csv", "r")
    data = list(csv.reader(file, delimiter=","))
    file.close()
    file2 = open("racetrack-database-master/racelines/Austin.csv", "r")
    data2 = list(csv.reader(file2, delimiter=","))
    file2.close()
    waypoints =[]
    for i in range(1,len(data2)-1):
        cx = float(data2[i][0])
        cy = float(data2[i][1])
        w_right = float(data[i][2])
        w_left = float(data[i][3])
        waypoints.append((cx,cy,w_right,w_left))  
        #pdb.set_trace()
    return waypoints   


def load_map2():
    mat = scipy.io.loadmat('track2.mat') 
    mat = {k:v for k, v in mat.items() if k[0] != '_'}
    track_inner = mat['track2'][0][0][0]
    track_outer = mat['track2'][0][0][1]
    track_center= mat['track2'][0][0][2]
    waypoints = np.hstack((track_inner.transpose(),track_center.transpose(),track_outer.transpose()))
    return waypoints
if __name__ == '__main__':
    waypoints = load_map2()
    pdb.set_trace()
    flg, ax = plt.subplots(1)
    #pdb.set_trace()
    # plt.plot(x, y, "xb", label="input")
    plt.plot(waypoints[0:,0], waypoints[0:,1], "-r", label="spline")
    plt.plot(waypoints[0:,2], waypoints[0:,3], "-g", label="bd_spline_right")
    plt.plot(waypoints[0:,4], waypoints[0:,5], "-b", label="bd_spline_left")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    plt.show()
    