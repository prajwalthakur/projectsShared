#!/usr/bin/env python3  
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
from  get_pointcloud.msg import  Obsposelist
import pdb
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf_conversions
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float32
ar_marker_0 = np.asarray([[1.929],[-0.768]])    #TODO
ar_marker_2 = np.asarray([[2.748],[-1.475]])    #TODO
ar_marker_3 = np.asarray([[3.095],[-2.18]])    #TODO
ar_marker_6 = np.asarray([[2.352],[-3.149]])    #TODO
ar_marker_7 = np.asarray([[3.819],[-1.309]])    #TODO

def callback(data):

    x_pos  = data.pose.position.x
    y_pos = data.pose.position.y
    drone_position_xy = np.array([[x_pos],[y_pos]])
    euclidian_dist_0  = np.linalg.norm((drone_position_xy-ar_marker_0),axis=0)[0]
    euclidian_dist_2  = np.linalg.norm((drone_position_xy-ar_marker_2),axis=0)[0]
    euclidian_dist_3  = np.linalg.norm((drone_position_xy-ar_marker_3),axis=0)[0]
    euclidian_dist_6  = np.linalg.norm((drone_position_xy-ar_marker_6),axis=0)[0]
    euclidian_dist_7  = np.linalg.norm((drone_position_xy-ar_marker_7),axis=0)[0]
    rospy.loginfo("euclidian_dist_0: "+str(euclidian_dist_0))
    rospy.loginfo("euclidian_dist_2: "+str(euclidian_dist_2))
    rospy.loginfo("euclidian_dist_3: "+str(euclidian_dist_3))
    rospy.loginfo("euclidian_dist_6: "+str(euclidian_dist_6))
    rospy.loginfo("euclidian_dist_7: "+str(euclidian_dist_7))
    drone_dist_publisher0.publish(drone_dist_publisher0)
    drone_dist_publisher2.publish(drone_dist_publisher2)
    drone_dist_publisher3.publish(drone_dist_publisher3)
    drone_dist_publisher6.publish(drone_dist_publisher6)
    drone_dist_publisher7.publish(drone_dist_publisher7)

if __name__ == '__main__':
    rospy.init_node('bebop_marker_diatance')
    drone_dist_publisher0 = rospy.Publisher("bebop_marker_distance_0", Float32, queue_size = 1)
    drone_dist_publisher2 = rospy.Publisher("bebop_marker_distance_2", Float32, queue_size = 1)
    drone_dist_publisher3 = rospy.Publisher("bebop_marker_distance_3", Float32, queue_size = 1)
    drone_dist_publisher6 = rospy.Publisher("bebop_marker_distance_6", Float32, queue_size = 1)
    drone_dist_publisher7 = rospy.Publisher("bebop_marker_distance_7", Float32, queue_size = 1)
    rospy.Subscriber("/vrpn_client_node/Bebop/pose",PoseStamped , callback)
    rospy.spin()