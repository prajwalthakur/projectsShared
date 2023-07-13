#!/usr/bin/env python3 
import rospy
import tf
from geometry_msgs.msg import Twist
import numpy as np
yaw = 0
pitch = 0
pi =3.14
def camera_get_angle_callback(msg):
    # this function extract teh current camera pitch and yaw from msg
    global yaw , pitch
    pitch = #TODO #get pitch value from msg
    yaw = #TODO #get pitch value from msg

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    #TODO create a subsrciber to /bebop/camera_control topic , message type : Twist , having queue_size=5 and camera_get_angle_callback as a call back function 
    r = rospy.Rate(100) #100hz do not edit this line
    while not rospy.is_shutdown(): 
        # TODO create a transformation having following properties
        # parent frame = Bebop
        #child frame = camera_optical
        #Transformation values : 
        # position displacement from bebop to camera_optical = (x axis displacement =10 cm ,  y axis displacement =0 cm, z axis displacement =0 cm)
        # Angular displacement from bebop to camera_optical = ( roll = -pi/2 + camera current pitch , pitch =0 , yaw = -pi/2 - camera current yaw)
        #for help see link : http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        r.sleep()
