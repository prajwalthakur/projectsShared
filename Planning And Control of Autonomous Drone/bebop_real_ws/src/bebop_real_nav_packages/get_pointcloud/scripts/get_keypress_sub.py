#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################

#48,49,50,51,52,53,54,55,56,57
def callback(data):
    rospy.loginfo(str(data))# to print on  terminal 


#s=115,e=101,g=103,b=98

if __name__=='__main__':
        rospy.init_node('get_keypress',anonymous=False)
        rospy.Subscriber("/obs_pose_list", Int8 ,callback )
        rospy.spin()