import numpy as np
import pdb

# return arc-length (theta) which the car has been traversed and closestIndex
def findTheta(currentPose,TrackCenter,theta_coordinates,trackWidth,last_closestIdx):    
    #theta_coordinates has the arc-lenghth correspoding to the index
    x_current = currentPose[0,0][0]
    y_current = currentPose[0,1][0]
    currentPosition = np.asarray([x_current,y_current])
    x_center_track = TrackCenter[0:,0]
    y_center_track = TrackCenter[0:,1]
    track_Center = TrackCenter #np.array[nx2]
    N_track = len(TrackCenter)
    
    #length of local search region
    N_search_region = 30
    search_region = np.zeros((N_search_region,))
    k=0
    for i in np.arange(last_closestIdx-5, last_closestIdx+20):
        search_region[k] = i    
        k+=1
    # path is circular , need to wrap 
    if last_closestIdx>20 or last_closestIdx<5 :
        i = np.where(search_region<0)[0]
        k = np.where(search_region>=N_track)
        search_region[i] = search_region[i] + N_track
        search_region[k] = search_region[k] - N_track
    
    #compute euclid distance of above potential points to the car
   
    search_region = search_region.astype(int)
    trackXsmall = x_center_track[search_region]
    trackYsmall = y_center_track[search_region]
    distanceX = trackXsmall - x_current*np.ones((search_region.shape[0],))
    distaceY = trackYsmall - y_current*np.ones((search_region.shape[0],))
    squared_distance = distanceX**2 + distaceY**2
    minIndex = squared_distance.argmin()
    e = squared_distance[minIndex]

    #if the distance is too large , then need to perform global search wrt to track width
    if (np.sqrt(e) > (trackWidth*1.25)/2):
        distanceX2 = x_center_track - x_current*np.ones((N_track,))
        distanceY2 = y_center_track - y_current*np.ones((N_track,))

        squared_distance_array2   = distanceX2**2 + distanceY2**2

        minIndex = np.argmin(squared_distance_array2)
        e = squared_distance_array2[minIndex]

    #circular edge conditions
    if(minIndex == 1):
        nextIdx = 2
        prevIdx = N_track
    elif(minIndex == N_track):
        nextIdx = 1
        prevIdx = N_track-1
    else:
        nextIdx = minIndex + 1
        prevIdx = minIndex - 1
    
    #Compute theta ( arc length that the car has traversed ) based on inner product projection
    closestIdx = minIndex
    #pdb.set_trace()
    cosinus = np.dot(currentPosition - TrackCenter[closestIdx,:] , TrackCenter[prevIdx,:] - TrackCenter[closestIdx,:])
    if(cosinus > 0):
        minIndex2 = prevIdx
    else:
        minIndex2 = minIndex
        minIndex = nextIdx
    
    if (e != 0) :
        cosinus = np.dot(currentPosition - TrackCenter[minIndex2,:] , TrackCenter[minIndex,:] - TrackCenter[minIndex2,:])/(np.linalg.norm(currentPosition - TrackCenter[minIndex2,:])*np.linalg.norm(TrackCenter[minIndex,:] - TrackCenter[minIndex2,:]))
    else:
        cosinus =0
    traj_breaks = np.where(theta_coordinates[:,0]==minIndex2)
    theta_k = theta_coordinates[traj_breaks,3]
    theta = theta_k + cosinus*np.linalg.norm(currentPosition - TrackCenter[minIndex2,:])
    return theta[0][0],closestIdx

if __name__ == '__main__':
    findTheta()