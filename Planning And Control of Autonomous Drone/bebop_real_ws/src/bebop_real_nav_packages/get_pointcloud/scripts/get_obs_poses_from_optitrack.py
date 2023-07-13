#!/usr/bin/env python  
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
from geometry_msgs.msg import PoseStamped
import message_filters
import pdb
from  get_pointcloud.msg import  Obsposelist

def get_transform_data(data):
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "obstacle_1"
    transform.transform.translation.x = data.pose.position.x
    transform.transform.translation.y = data.pose.position.y
    transform.transform.translation.z = data.pose.position.z
    transform.transform.rotation.w = data.pose.orientation.w
    transform.transform.rotation.x = data.pose.orientation.x
    transform.transform.rotation.y = data.pose.orientation.y
    transform.transform.rotation.z = data.pose.orientation.z  
    return transform

if __name__ == '__main__':
    rospy.init_node('obs_optitrack_pose_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    publish_frequency = rospy.get_param("publish_frequency", 10)
    obs_list_pub = rospy.Publisher("/obs_pose_list", Obsposelist ,queue_size=1)
    last_published = None
    def callback(obs1_pose):
        global last_published
        if last_published and publish_frequency > 0.0 and time.time() - last_published <= 1.0 / publish_frequency:
            return
        transforms = []
        obstacles_list =Obsposelist()
        transform1 = get_transform_data(obs1_pose)
        obstacles_list.obs_poses_list.append(transform1)  
        transforms.append(transform1)
        
        # transform2 = get_transform_data(obs2_pose)
        # obstacles_list.obs_poses_list.append(transform2)  
        # transforms.append(transform2)
        
        # transform3 = get_transform_data(obs3_pose)
        # obstacles_list.obs_poses_list.append(transform3)  
        # transforms.append(transform3)

        broadcaster.sendTransform(transforms)
        last_published = time.time()
        
        obs_list_pub.publish(obstacles_list)


    obs1_pose = rospy.Subscriber("/vrpn_client_node/obstacle_1/pose", PoseStamped, callback)
    # obs2_pose = message_filters.Subscriber("", PoseStamped)
    # obs3_pose = message_filters.Subscriber("", PoseStamped)
    # ts = message_filters.ApproximateTimeSynchronizer([obs1_pose], 1,1, allow_headerless=True)
    # ts.registerCallback(callback)
    rospy.spin()