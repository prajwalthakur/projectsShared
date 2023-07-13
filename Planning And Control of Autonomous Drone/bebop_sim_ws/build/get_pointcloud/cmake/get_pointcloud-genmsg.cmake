# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "get_pointcloud: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iget_pointcloud:/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(get_pointcloud_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" NAME_WE)
add_custom_target(_get_pointcloud_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "get_pointcloud" "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" "geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3:geometry_msgs/TransformStamped:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(get_pointcloud
  "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/get_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_cpp(get_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/get_pointcloud
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(get_pointcloud_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(get_pointcloud_generate_messages get_pointcloud_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" NAME_WE)
add_dependencies(get_pointcloud_generate_messages_cpp _get_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(get_pointcloud_gencpp)
add_dependencies(get_pointcloud_gencpp get_pointcloud_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS get_pointcloud_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(get_pointcloud
  "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/get_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_eus(get_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/get_pointcloud
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(get_pointcloud_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(get_pointcloud_generate_messages get_pointcloud_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" NAME_WE)
add_dependencies(get_pointcloud_generate_messages_eus _get_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(get_pointcloud_geneus)
add_dependencies(get_pointcloud_geneus get_pointcloud_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS get_pointcloud_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(get_pointcloud
  "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/get_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_lisp(get_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/get_pointcloud
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(get_pointcloud_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(get_pointcloud_generate_messages get_pointcloud_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" NAME_WE)
add_dependencies(get_pointcloud_generate_messages_lisp _get_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(get_pointcloud_genlisp)
add_dependencies(get_pointcloud_genlisp get_pointcloud_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS get_pointcloud_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(get_pointcloud
  "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/get_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_nodejs(get_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/get_pointcloud
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(get_pointcloud_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(get_pointcloud_generate_messages get_pointcloud_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" NAME_WE)
add_dependencies(get_pointcloud_generate_messages_nodejs _get_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(get_pointcloud_gennodejs)
add_dependencies(get_pointcloud_gennodejs get_pointcloud_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS get_pointcloud_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(get_pointcloud
  "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/get_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_py(get_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/get_pointcloud
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(get_pointcloud_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(get_pointcloud_generate_messages get_pointcloud_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/get_pointcloud/msg/Obsposelist.msg" NAME_WE)
add_dependencies(get_pointcloud_generate_messages_py _get_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(get_pointcloud_genpy)
add_dependencies(get_pointcloud_genpy get_pointcloud_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS get_pointcloud_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/get_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/get_pointcloud
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(get_pointcloud_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(get_pointcloud_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(get_pointcloud_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/get_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/get_pointcloud
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(get_pointcloud_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(get_pointcloud_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(get_pointcloud_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/get_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/get_pointcloud
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(get_pointcloud_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(get_pointcloud_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(get_pointcloud_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/get_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/get_pointcloud
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(get_pointcloud_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(get_pointcloud_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(get_pointcloud_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/get_pointcloud)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/get_pointcloud\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/get_pointcloud
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(get_pointcloud_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(get_pointcloud_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(get_pointcloud_generate_messages_py geometry_msgs_generate_messages_py)
endif()
