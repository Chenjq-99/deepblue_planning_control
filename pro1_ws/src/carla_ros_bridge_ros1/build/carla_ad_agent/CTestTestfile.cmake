# CMake generated Testfile for 
# Source directory: /home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/src/carla_ad_agent
# Build directory: /home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/build/carla_ad_agent
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_carla_ad_agent_roslaunch-check_launch "/home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/build/carla_ad_agent/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/build/carla_ad_agent/test_results/carla_ad_agent/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/build/carla_ad_agent/test_results/carla_ad_agent" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/build/carla_ad_agent/test_results/carla_ad_agent/roslaunch-check_launch.xml\" \"/home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/src/carla_ad_agent/launch\" ")
set_tests_properties(_ctest_carla_ad_agent_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/src/carla_ad_agent/CMakeLists.txt;16;roslaunch_add_file_check;/home/qrq/Documents/Shenlan_PnC/carla_ros_bridge_ros1/src/carla_ad_agent/CMakeLists.txt;0;")
subdirs("gtest")
