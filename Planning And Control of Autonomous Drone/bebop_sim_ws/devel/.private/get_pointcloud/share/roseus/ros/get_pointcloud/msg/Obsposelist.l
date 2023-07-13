;; Auto-generated. Do not edit!


(when (boundp 'get_pointcloud::Obsposelist)
  (if (not (find-package "GET_POINTCLOUD"))
    (make-package "GET_POINTCLOUD"))
  (shadow 'Obsposelist (find-package "GET_POINTCLOUD")))
(unless (find-package "GET_POINTCLOUD::OBSPOSELIST")
  (make-package "GET_POINTCLOUD::OBSPOSELIST"))

(in-package "ROS")
;;//! \htmlinclude Obsposelist.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass get_pointcloud::Obsposelist
  :super ros::object
  :slots (_obs_poses_list ))

(defmethod get_pointcloud::Obsposelist
  (:init
   (&key
    ((:obs_poses_list __obs_poses_list) ())
    )
   (send-super :init)
   (setq _obs_poses_list __obs_poses_list)
   self)
  (:obs_poses_list
   (&rest __obs_poses_list)
   (if (keywordp (car __obs_poses_list))
       (send* _obs_poses_list __obs_poses_list)
     (progn
       (if __obs_poses_list (setq _obs_poses_list (car __obs_poses_list)))
       _obs_poses_list)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/TransformStamped[] _obs_poses_list
    (apply #'+ (send-all _obs_poses_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/TransformStamped[] _obs_poses_list
     (write-long (length _obs_poses_list) s)
     (dolist (elem _obs_poses_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/TransformStamped[] _obs_poses_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obs_poses_list (let (r) (dotimes (i n) (push (instance geometry_msgs::TransformStamped :init) r)) r))
     (dolist (elem- _obs_poses_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get get_pointcloud::Obsposelist :md5sum-) "0aaa214c546cd6dd8dd28dc01d8ebbac")
(setf (get get_pointcloud::Obsposelist :datatype-) "get_pointcloud/Obsposelist")
(setf (get get_pointcloud::Obsposelist :definition-)
      "geometry_msgs/TransformStamped[] obs_poses_list
================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :get_pointcloud/Obsposelist "0aaa214c546cd6dd8dd28dc01d8ebbac")


