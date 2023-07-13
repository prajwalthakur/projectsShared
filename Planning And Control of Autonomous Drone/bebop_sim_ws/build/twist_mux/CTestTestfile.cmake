# CMake generated Testfile for 
# Source directory: /home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/hector_neotic/twist_mux-melodic-devel
# Build directory: /home/prajwal20/ims/droneCourse/bebop_sim_ws/build/twist_mux
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_twist_mux_rostest_test_system.test "/home/prajwal20/ims/droneCourse/bebop_sim_ws/build/twist_mux/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/prajwal20/ims/droneCourse/bebop_sim_ws/build/twist_mux/test_results/twist_mux/rostest-test_system.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/hector_neotic/twist_mux-melodic-devel --package=twist_mux --results-filename test_system.xml --results-base-dir \"/home/prajwal20/ims/droneCourse/bebop_sim_ws/build/twist_mux/test_results\" /home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/hector_neotic/twist_mux-melodic-devel/test/system.test ")
set_tests_properties(_ctest_twist_mux_rostest_test_system.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/hector_neotic/twist_mux-melodic-devel/CMakeLists.txt;56;add_rostest;/home/prajwal20/ims/droneCourse/bebop_sim_ws/src/bebop_navigation_simulationnew/hector_neotic/twist_mux-melodic-devel/CMakeLists.txt;0;")
subdirs("gtest")
