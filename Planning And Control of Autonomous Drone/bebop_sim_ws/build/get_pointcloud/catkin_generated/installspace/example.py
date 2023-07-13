#!/usr/bin/env python3
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

def publish_pointcloud(pcl_pub):
        
    '''
    Sample code to publish a pcl2 with python
    '''       
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    
    cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'bebopbase_footprint'
    header.frame_id = 'map'
    #create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    #publish    
    rospy.loginfo("happily publishing sample pointcloud.. !")
    pcl_pub.publish(scaled_polygon_pcl)
    
    
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('pcl2_pub_example')
        pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
        rospy.sleep(1.)
        while not rospy.is_shutdown() :            
            publish_pointcloud(pcl_pub)
    except rospy.ROSInterruptException:  pass
    