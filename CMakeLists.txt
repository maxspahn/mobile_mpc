cmake_minimum_required(VERSION 3.0.2)
project(mobile_mpc)

## Switch between the static and dynamic mpc-formulation
if(NOT DEFINED STATIC_MPC)
  set(STATIC_MPC 0)
endif(NOT DEFINED STATIC_MPC)

MESSAGE(STATUS ${STATIC_MPC})

add_definitions(-DSTATIC_MPC=${STATIC_MPC})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
    mpc.action
    simpleMpc.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    mm_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
include_directories(
  include
  #acado/generated_mpc
  #acado/generated_mpc/qpoases/INCLUDE
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  forcesLib/simpleMPC/simplempc/include
)
if(STATIC_MPC EQUAL 1)
  include_directories(forcesLib/mobileManipulator/static_spheres/mm_MPC/include)
else(STATIC_MPC EQUAL 1)
  include_directories(forcesLib/mobileManipulator/decomp_util/mm_MPC/include)
endif(STATIC_MPC EQUAL 1)

  

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mobile_mpc.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mobile_mpc_node.cpp)

add_library(mpc_lib
#  src/acado_converter.cpp
  src/forces_converter.cpp
  src/mpc_problem.cpp
#  src/mpc_acado_solver.cpp
  src/mpc_forces_solver.cpp
  src/mpc_interface.cpp
)

if(STATIC_MPC EQUAL 1)
  target_link_libraries(mpc_lib
    ${CMAKE_SOURCE_DIR}/forcesLib/mobileManipulator/static_spheres/mm_MPC/lib/libmm_MPC_withModel.so
    ${catkin_LIBRARIES}
  )
else(STATIC_MPC EQUAL 1)
  target_link_libraries(mpc_lib
    ${CMAKE_SOURCE_DIR}/forcesLib/mobileManipulator/decomp_util/mm_MPC/lib/libmm_MPC_withModel.so
    ${catkin_LIBRARIES}
  )
endif(STATIC_MPC EQUAL 1)

add_executable(mpc_server
  src/mpc_action_server.cpp
)
target_link_libraries(mpc_server
  mpc_lib
  ${catkin_LIBRARIES}
)

## testing
catkin_add_gtest(mpc_problem_test test/mpc_problem_test.cpp)
target_link_libraries(mpc_problem_test mpc_lib)
##catkin_add_gtest(acado_converter_test test/acado_converter_test.cpp)
##target_link_libraries(acado_converter_test mpc_lib)
catkin_add_gtest(forces_converter_test test/forces_converter_test.cpp)
target_link_libraries(forces_converter_test mpc_lib)
#catkin_add_gtest(mpc_acado_solver_test test/mpc_acado_solver_test.cpp)
#target_link_libraries(mpc_acado_solver_test mpc_lib)
catkin_add_gtest(mpc_forces_solver_test test/mpc_forces_solver_test.cpp)
target_link_libraries(mpc_forces_solver_test mpc_lib)

# Simple isolated planner 
add_executable(mpcplanner
  src/mpcPlanner.cpp
)

target_link_libraries(mpcplanner
  ${CMAKE_SOURCE_DIR}/forcesLib/simpleMPC/simplempc/lib/libsimplempc_withModel.so
  ${catkin_LIBRARIES}
)
  

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
