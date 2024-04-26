cmake_minimum_required(VERSION 3.0.2)
project(antenna_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  messages
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS
  include
  /usr/local/include/libpololu-tic-1
 LIBRARIES
  pololu-tic-1
  tic_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/local/include/libpololu-tic-1
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# link_directories(${Eigen3_INCLUDE_DIRS})

## Declare a C++ library
add_library(tic_lib
  src/tic_lib.cpp
)

add_library(motor_predictor_lib
  src/predictor_lib.cpp
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tic_collect_data src/tic_collect_data_node.cpp)
add_executable(motor_pose src/motor_pose_node.cpp)
add_executable(motor_control src/motor_control_node.cpp)
add_executable(antenna_pose_node src/antenna_pose_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tic_collect_data
  ${catkin_LIBRARIES} 
  pololu-tic-1
  tic_lib 
)

target_link_libraries(motor_pose
  ${catkin_LIBRARIES} 
  pololu-tic-1
  tic_lib 
)

target_link_libraries(motor_control
  ${catkin_LIBRARIES} 
  pololu-tic-1
  tic_lib 
  motor_predictor_lib
)

target_link_libraries(antenna_pose_node
  ${catkin_LIBRARIES} 
)
# target_link_libraries(tic_predictor
#   ${Eigen3_LIBRARIES}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_antenna_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

add_executable(predictor_lib_test test/predictor_lib_test.cpp)
target_link_libraries(predictor_lib_test
  motor_predictor_lib
)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
