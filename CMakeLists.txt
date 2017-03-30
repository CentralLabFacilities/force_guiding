cmake_minimum_required(VERSION 2.8.3)
project(force_guiding)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        std_msgs
        geometry_msgs
        roslaunch
        dynamic_reconfigure
        message_generation
	actionlib
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Velocity.srv
)

add_action_files(
	DIRECTORY action 
	FILES Guiding.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
        cfg/Module.cfg
        cfg/Controller.cfg
)

##################
## launch files ##
##################

roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES force_guiding
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(force_guiding src/guiding.cpp src/MovementController.cpp src/MovementModule.cpp)
add_executable(stiffness_publisher src/stiffness_publisher.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(force_guiding ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)
add_dependencies(stiffness_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(force_guiding
        ${catkin_LIBRARIES}
)

target_link_libraries(stiffness_publisher
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} stiffness_publisher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY share
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install rosjava messages
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/rosjava_messages/ 
DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/rosjava_messages)
