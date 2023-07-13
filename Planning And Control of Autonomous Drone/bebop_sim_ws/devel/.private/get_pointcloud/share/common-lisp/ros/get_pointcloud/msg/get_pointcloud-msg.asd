
(cl:in-package :asdf)

(defsystem "get_pointcloud-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Obsposelist" :depends-on ("_package_Obsposelist"))
    (:file "_package_Obsposelist" :depends-on ("_package"))
  ))