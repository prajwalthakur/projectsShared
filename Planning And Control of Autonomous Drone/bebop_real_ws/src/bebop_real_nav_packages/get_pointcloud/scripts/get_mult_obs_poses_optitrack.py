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

def get_transform_data(data,child_frame_id_string):
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = child_frame_id_string
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
    def callback(obs1_pose,obs2_pose): #add obs3_pose argument too, if you have three obstacle
        global last_published
        if last_published and publish_frequency > 0.0 and time.time() - last_published <= 1.0 / publish_frequency:
            return
        transforms = []
        obstacles_list =Obsposelist()
        child_frame_id_string = "obstacle_1"
        transform1 = get_transform_data(obs1_pose,child_frame_id_string)
        obstacles_list.obs_poses_list.append(transform1)  
        transforms.append(transform1)
        
        #uncomment next four lines too, if you have two obstacles
        child_frame_id_string = "obstacle_2"
        transform2 = get_transform_data(obs2_pose,child_frame_id_string)
        obstacles_list.obs_poses_list.append(transform2)  
        transforms.append(transform2)
        
        #uncommnet next four lines too ,if you have three obstalces
        #child_frame_id_string = "obstacle_3"
        # transform3 = get_transform_data(obs3_pos,child_frame_id_stringe)
        # obstacles_list.obs_poses_list.append(transform3)  
        # transforms.append(transform3)

        broadcaster.sendTransform(transforms)
        last_published = time.time()        
        obs_list_pub.publish(obstacles_list)


    #obs1_pose = rospy.Subscriber("/vrpn_client_node/obstacle_1/pose", PoseStamped, callback)
    obs1_pose = message_filters.Subscriber("vrpn_client_node/obstacle_1/pose", PoseStamped)
    obs2_pose = message_filters.Subscriber("vrpn_client_node/obstacle_2/pose", PoseStamped)  #uncomment this line too, if you have two obstacles
    # obs3_pose = message_filters.Subscriber("vrpn_client_node/obstacle_3/pose", PoseStamped)  #uncommnet this line too ,if you have three obstacles
    ts = message_filters.ApproximateTimeSynchronizer([obs1_pose,obs2_pose],1, allow_headerless=True,slop=0.1)  #add obs3_pose if you have three obstacles
    ts.registerCallback(callback)
    rospy.spin()