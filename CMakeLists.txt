cmake_minimum_required(VERSION 2.8.3)
project(finger_fk)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  hrl_kdl
  hrl_geom
  pykdl_utils
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FingerFkService.srv
	FingerJointsService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
	geometry_msgs
	sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


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
#  INCLUDE_DIRS include
#  LIBRARIES finger_fk
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs hrl_kdl hrl_geom pykdl_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(finger_joints_pub src/finger_joints_pub.cpp)
add_executable(finger_joints_service src/finger_joints_service.cpp)
add_executable(test_finger_joints_service test/test_finger_joints_service.cpp)

target_link_libraries(finger_joints_pub
   ${catkin_LIBRARIES}
)
target_link_libraries(finger_joints_service
   ${catkin_LIBRARIES}
)
target_link_libraries(test_finger_joints_service
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
