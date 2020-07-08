cmake_minimum_required(VERSION 2.8)

project(acado_mpc_generation)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

find_package(ACADO REQUIRED)

include_directories( ${ACADO_INCLUDE_DIRS} )
message(${ACADO_INCLUDE_DIRS})

add_executable(my_mpc src/mpc_acado.cpp) 
target_link_libraries(my_mpc ${ACADO_SHARED_LIBRARIES} )

add_executable(mpc_test src/mpc_test.cpp) 
target_link_libraries(mpc_test ${ACADO_SHARED_LIBRARIES} )

message(${ACADO_SHARED_LIBRARIES})

set_target_properties(
  my_mpc PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(
  mpc_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
