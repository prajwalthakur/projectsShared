#!/usr/bin/env python3 

import rospy
import tf
from geometry_msgs.msg import Twist
import numpy as np
yaw = 0
pitch = 0
pi =3.14
def camera_tilt_tf_callback(msg):
    global yaw , pitch
    pitch = np.radians(msg.angular.y)
    yaw = np.radians(msg.angular.z)
    # rospy.loginfo(f"\t\nmsg.angular.y:\n{msg.angular.y}[deg]")

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rospy.Subscriber("/bebop/camera_control", Twist, camera_tilt_tf_callback, queue_size=1) 
    r = rospy.Rate(100) #100hz
    while not rospy.is_shutdown(): 
        br.sendTransform((0.1, 0, 0),
                    tf.transformations.quaternion_from_euler(-1.5707963268 + pitch, 0, -1.5707963268-yaw),
                    rospy.Time.now(),
                    "camera_optical",
                    "Bebop2")
        rospy.loginfo(f"\t\nmsg.angular.y:\n{yaw*180/pi}[degree]")
        r.sleep()
    #rospy.spin()