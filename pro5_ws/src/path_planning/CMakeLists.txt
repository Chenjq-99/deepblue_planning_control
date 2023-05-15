cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions("-Wall -g")

# find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  # lgsvl_msgs
  carla_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ros_viz_tools
  base_local_planner
)

catkin_package(
  LIBRARIES serial_communication
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf carla_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

link_directories(${catkin_LIBRARIES} lib)

add_library(path_planning
            src/lqr_controller.cpp
            src/reference_line.cpp
            src/pid_controller.cpp
            # src/trajectory_planner.cpp
            # src/trajectory_planner_utils.cpp
            # src/cubic_spline_planner.cpp
            src/frenet_optimal_trajectory.cpp
            src/visualization.cpp
          )
               

target_link_libraries(path_planning ${catkin_LIBRARIES} VTSMapInterfaceCPP)

add_executable(path_planning_node src/main.cpp src/path_planning_node.cpp)
target_link_libraries(path_planning_node path_planning)