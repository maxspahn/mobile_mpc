cmake_minimum_required(VERSION 2.8)

project(acado_mpc_generation)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

find_package(ACADO REQUIRED)

include_directories( ${ACADO_INCLUDE_DIRS} )

add_executable(my_mpc src/mpc_acado.cpp) 
target_link_libraries(my_mpc ${ACADO_SHARED_LIBRARIES} )

message(${ACADO_SHARED_LIBRARIES})

set_target_properties(
  my_mpc PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
