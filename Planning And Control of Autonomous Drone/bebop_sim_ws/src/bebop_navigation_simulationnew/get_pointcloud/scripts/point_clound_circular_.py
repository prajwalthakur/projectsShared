#!/usr/bin/env python3
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import geometry_msgs.msg
import numpy as np
import pdb
from  get_pointcloud.msg import  Obsposelist
z = 1.0
radius = 0.25
pcl_pub = None
def create_a_circle(center_x,center_y):
    list_circular_point= []
    points = np.arange(0,2*np.pi,np.pi/180)
    x_points = center_x + radius*np.cos(points)
    x_points = x_points.reshape(x_points.shape[0],1)
    y_points = center_y + radius*np.sin(points)
    y_points = y_points.reshape(y_points.shape[0],1)
    z_points = z*np.ones(x_points.shape[0])
    z_points = z_points.reshape(z_points.shape[0],1)
    points = np.hstack((x_points,y_points,z_points))
    #list_circular_point = points.tolist()    
    return points

def publish_pointcloud(obs_pose ):
    global pcl_pub   
    '''
    Sample code to publish a pcl2 with python
    '''       
    #rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    list_circular_point_array = np.zeros((1,1))
    for i in range(len(obs_pose.obs_poses_list)):
        center_x = obs_pose.obs_poses_list[i].transform.translation.x
        center_y = obs_pose.obs_poses_list[i].transform.translation.y
        if i==0:            
            list_circular_point_array = create_a_circle(center_x,center_y)
        else :
            temp = create_a_circle(center_x,center_y)
            list_circular_point_array = np.vstack((list_circular_point_array,temp))
        
    list_circular_point_list = list_circular_point_array.tolist()
    # pdb.set_trace()
    
    
    
    cloud_points =  list_circular_point_list #[[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    #header.frame_id = 'bebopbase_footprint'
    header.frame_id = 'world'
    #create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    #publish    
    # rospy.loginfo(f"{obs_pose.obs_poses_list[-1].transform.translation.x}")
    pcl_pub.publish(scaled_polygon_pcl)
    
    
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('pcl2_obs_pub_example')
        pcl_pub = rospy.Publisher("/obs1_pcl_topic", PointCloud2) 
        r = rospy.Rate(10) # 10hz
        rospy.Subscriber("/obs_pose_list", Obsposelist ,publish_pointcloud )
        rospy.spin()
    except rospy.ROSInterruptException:  pass
    