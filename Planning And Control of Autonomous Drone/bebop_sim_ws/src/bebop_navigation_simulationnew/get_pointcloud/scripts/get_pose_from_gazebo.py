#!/usr/bin/env python  
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
from  get_pointcloud.msg import  Obsposelist
import pdb

if __name__ == '__main__':
    rospy.init_node('gazebo_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    publish_frequency = rospy.get_param("publish_frequency", 10)
    obs_pose_pub1 = rospy.Publisher("/obs_pose_1", geometry_msgs.msg.TransformStamped, queue_size=1 )
    obs_pose_pub2 = rospy.Publisher("/obs_pose_2", geometry_msgs.msg.TransformStamped ,queue_size=1)
    obs_pose_pub3 = rospy.Publisher("/obs_pose_3", geometry_msgs.msg.TransformStamped ,queue_size=1)
    obs_list_pub = rospy.Publisher("/obs_pose_list", Obsposelist ,queue_size=1)
    last_published = None
    def callback(data):
        global last_published
        if last_published and publish_frequency > 0.0 and time.time() - last_published <= 1.0 / publish_frequency:
            return
        transforms = []
        obstacles_list =Obsposelist()
        for i in range(len(data.name)):
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "world"
            transform.child_frame_id = data.name[i]
            transform.transform.translation.x = data.pose[i].position.x
            transform.transform.translation.y = data.pose[i].position.y
            transform.transform.translation.z = data.pose[i].position.z
            transform.transform.rotation.w = data.pose[i].orientation.w
            transform.transform.rotation.x = data.pose[i].orientation.x
            transform.transform.rotation.y = data.pose[i].orientation.y
            transform.transform.rotation.z = data.pose[i].orientation.z
            transforms.append(transform)
            
            #pdb.set_trace()
            if data.name[i] =='obstacle_1':
                obs_pose_pub1.publish(transform)
                obstacles_list.obs_poses_list.append(transform)
            if data.name[i] == 'obstacle_2':
                obs_pose_pub2.publish(transform)
                obstacles_list.obs_poses_list.append(transform)
            if data.name[i] == 'obstacle_3':
                obs_pose_pub3.publish(transform)
                obstacles_list.obs_poses_list.append(transform)
        broadcaster.sendTransform(transforms)
        obs_list_pub.publish(obstacles_list)
        last_published = time.time()

    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)

    rospy.spin()