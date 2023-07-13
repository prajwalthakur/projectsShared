 global is_received, vehicle_pose_vel, drone_pose_vel, odom_mutex
    odom_mutex.acquire()
    vehicle_orientation_q = vehicle_odom.pose.pose.orientation
    vehicle_orientation_list = [vehicle_orientation_q.x, vehicle_orientation_q.y, vehicle_orientation_q.z, vehicle_orientation_q.w]
    drone_orientation_q = drone_odom.pose.pose.orientation
    drone_orientation_list = [drone_orientation_q.x, drone_orientation_q.y, drone_orientation_q.z, drone_orientation_q.w]
#     obs_1_orientation_q = obs_1_odom.pose.pose.orientation
#     obs_1_orientation_list = [obs_1_orientation_q.x, obs_1_orientation_q.y, obs_1_orientation_q.z, obs_1_orientation_q.w]
#     obs_2_orientation_q = obs_2_odom.pose.pose.orientation
#     obs_2_orientation_list = [obs_2_orientation_q.x, obs_2_orientation_q.y, obs_2_orientation_q.z, obs_2_orientation_q.w]
#     obs_3_orientation_q = obs_3_odom.pose.pose.orientation
#     obs_3_orientation_list = [obs_3_orientation_q.x, obs_3_orientation_q.y, obs_3_orientation_q.z, obs_3_orientation_q.w]
    drone_orientation_q = drone_odom.pose.pose.orientation
    drone_orientation_list = [drone_orientation_q.x, drone_orientation_q.y, drone_orientation_q.z, drone_orientation_q.w]
    (robot_roll, robot_pitch, vehicle_yaw) = euler_from_quaternion (vehicle_orientation_list)
    (target_roll, target_pitch, drone_yaw) = euler_from_quaternion (drone_orientation_list)
#     (obs_1_roll, obs_1_pitch, obs_1_yaw) = euler_from_quaternion (obs_1_orientation_list)
#     (obs_2_roll, obs_2_pitch, obs_2_yaw) = euler_from_quaternion (obs_2_orientation_list)
#     (obs_3_roll, obs_3_pitch, obs_3_yaw) = euler_from_quaternion (obs_3_orientation_list)
    vehicle_pose_vel = [vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y, vehicle_yaw,
                    vehicle_odom.twist.twist.linear.x, vehicle_odom.twist.twist.linear.y, vehicle_odom.twist.twist.angular.z]
    drone_pose_vel = [drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z ,drone_yaw,
                    drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.linear.z, drone_odom.twist.twist.angular.z]
#     obs_1_pose_vel = [obs_1_odom.pose.pose.position.x, obs_1_odom.pose.pose.position.y, obs_1_yaw,
#                     obs_1_odom.twist.twist.linear.x, obs_1_odom.twist.twist.linear.y, obs_1_odom.twist.twist.angular.z]
#     obs_2_pose_vel = [obs_2_odom.pose.pose.position.x, obs_2_odom.pose.pose.position.y, obs_2_yaw,
#                     obs_2_odom.twist.twist.linear.x, obs_2_odom.twist.twist.linear.y, obs_2_odom.twist.twist.angular.z]
#     obs_3_pose_vel = [obs_3_odom.pose.pose.position.x, obs_3_odom.pose.pose.position.y, obs_3_yaw,
#                     obs_3_odom.twist.twist.linear.x, obs_3_odom.twist.twist.linear.y, obs_3_odom.twist.twist.angular.z]