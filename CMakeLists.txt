cmake_minimum_required(VERSION 3.0.2)
project(mobile_mpc)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
  genmsg
  mm_msgs
  trajectory_msgs
  moveit_msgs
)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
    simpleMpc.action
)

add_message_files(
  FILES
  LinearConstraint3D.msg
  LinearConstraint3DArray.msg
  DynamicObstacleMsg.msg
  ObjectMsg.msg
  NurbsEval2D.msg
  SolverInfo.msg
  SingleStaticSphere.msg
  StaticSphereMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    mm_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  forcesLib/simpleMPC/simplempc/include
  forcesLib/sphereMPC/sphere5/include
)

# Simple isolated planner 
add_executable(mpcplanner
  src/mpcPlanner.cpp
)

target_link_libraries(mpcplanner
  ${CMAKE_SOURCE_DIR}/forcesLib/simpleMPC/simplempc/lib/libsimplempc_withModel.so
  ${catkin_LIBRARIES}
)

# static planner 
add_executable(mpcsphereplanner
  src/mpcSpherePlanner.cpp
)

target_link_libraries(mpcsphereplanner
  ${CMAKE_SOURCE_DIR}/forcesLib/sphereMPC/sphere5/lib/libsphere5_withModel.so
  ${catkin_LIBRARIES}
)
  

# Free Region Decomposotion Node
add_executable(decomp_node
  src/decomp_node.cpp
)
target_link_libraries(decomp_node
  ${catkin_LIBRARIES}
)

add_executable(laserScan_assem
  src/laser_scan_assembler.cpp
)

target_link_libraries(laserScan_assem
  ${catkin_LIBRARIES}
)

# Activate Debug mode
# set(CMAKE_BUILD_TYPE Debug)
