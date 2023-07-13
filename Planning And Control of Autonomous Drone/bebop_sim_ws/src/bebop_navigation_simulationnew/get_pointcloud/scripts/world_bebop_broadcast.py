#!/usr/bin/env python  
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
import pdb

# if __name__ == '__main__':
#     rospy.init_node('gazebo_tf_broadcaster')

#     broadcaster = tf2_ros.StaticTransformBroadcaster()    
#     publish_frequency = rospy.get_param("publish_frequency", 10)
#     last_published = None
#     def callback(data):
#         global last_published
#         if last_published and publish_frequency > 0.0 and time.time() - last_published <= 1.0 / publish_frequency:
#             return
#         for i in range(len(data.name)):
#             transform = geometry_msgs.msg.TransformStamped()
#             transform.header.stamp = rospy.Time.now()
#             transform.header.frame_id = "world"
#             transform.child_frame_id = "bebopodom"
#             transform.transform.translation.x = data.pose[i].position.x
#             transform.transform.translation.y = data.pose[i].position.y
#             transform.transform.translation.z = data.pose[i].position.z
#             transform.transform.rotation.w = data.pose[i].orientation.w
#             transform.transform.rotation.x = data.pose[i].orientation.x
#             transform.transform.rotation.y = data.pose[i].orientation.y
#             transform.transform.rotation.z = data.pose[i].orientation.z
#         broadcaster.sendTransform(transform)
#         last_published = time.time()

#     rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)

#     rospy.spin()    






if __name__ == '__main__':

    rospy.init_node('world_bebop_tf_broadcast')
    r = rospy.Rate(10) # 10hz
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    while not rospy.is_shutdown() :
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "bebopodom"
        transform.transform.translation.x = -6.66908
        transform.transform.translation.y = 6.44567
        transform.transform.translation.z = 0.5
        transform.transform.rotation.w = 1
        transform.transform.rotation.x = 0
        transform.transform.rotation.y = 0
        transform.transform.rotation.z = 0
        broadcaster.sendTransform(transform)
        r.sleep()
