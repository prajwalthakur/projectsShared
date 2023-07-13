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

broadcaster = None
last_published = None
publish_frequency = None
odom_publisher = None

def get_odom_base_footprint(transform,data):
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x = data.pose.pose.position.x
    transform.transform.translation.y = data.pose.pose.position.y
    transform.transform.translation.z = 0
    # (drone_roll, drone_pitch, drone_yaw) = euler_from_quaternion([data.pose.orientation.w,data.pose.orientation.x, data.pose.orientation.y,
    #                                                     data.pose.orientation.z])
    # base_foot_print_quaternion = quaternion_from_euler( 0.0 , 0.0 , drone_yaw)
    transform.transform.rotation = data.pose.pose.orientation
    # transform.transform.rotation.x = 0#base_foot_print_quaternion[1]
    # transform.transform.rotation.y = 0#base_foot_print_quaternion[2]
    # transform.transform.rotation.z = 0#base_foot_print_quaternion[3]
    
    return transform
    

# def get_base_foot_print_bebop(transform2,data):
#     transform2.header.stamp = rospy.Time.now()
#     transform2.header.frame_id = "base_footprint"
#     transform2.child_frame_id = "Bebop_new"
#     transform2.transform.translation.x = 0
#     transform2.transform.translation.y = 0
#     transform2.transform.translation.z = data.pose.position.z
#     # (drone_roll, drone_pitch, drone_yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
#     #                                                     data.pose.orientation.z, data.pose.orientation.w])
#     base_foot_print_quaternion = quaternion_from_euler(data.pose.orientation.x, data.pose.orientation.y, 0)
#     transform2.transform.rotation.w = base_foot_print_quaternion[0]
#     transform2.transform.rotation.x = base_foot_print_quaternion[1]
#     transform2.transform.rotation.y = base_foot_print_quaternion[2]
#     transform2.transform.rotation.z =base_foot_print_quaternion[3]

def callback(data):
    global last_published, broadcaster, publish_frequency, odom_publisher
    transforms =[]
    (drone_roll, drone_pitch, drone_yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
                                                        data.pose.orientation.z, data.pose.orientation.w])
    bebop_orientation = quaternion_from_euler( 0.0 , 0.0 , drone_yaw)
    bebop_odom = Odometry()
    bebop_odom.header.frame_id = 'odom'
    bebop_odom.pose.pose.position = data.pose.position
    bebop_odom.pose.pose.orientation.x = bebop_orientation[0]
    bebop_odom.pose.pose.orientation.y = bebop_orientation[1]
    bebop_odom.pose.pose.orientation.z = bebop_orientation[2]
    bebop_odom.pose.pose.orientation.w = bebop_orientation[3]
    odom_publisher.publish(bebop_odom)

    transform = geometry_msgs.msg.TransformStamped()
    transform = get_odom_base_footprint(transform,bebop_odom)
    #transforms.append(transform)
    #transform2 = geometry_msgs.msg.TransformStamped()
    #get_base_foot_print_bebop(transform2,data)
    #transforms.append(transform2)
    broadcaster.sendTransform(transform)
    # last_published = time.time()

if __name__ == '__main__':
    rospy.init_node('drone_optitrack_pose_broadcaster')
    odom_publisher = rospy.Publisher("bebop_optitrack/odom", Odometry, queue_size = 1)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    publish_frequency = rospy.get_param("publish_frequency", 10)
    rospy.Subscriber("/vrpn_client_node/Bebop2/pose",PoseStamped , callback)

    rospy.spin()