cmake_minimum_required(VERSION 2.8.3)
project(plan_reparametrizer)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  moveit_msgs
  moveit_core
  geometric_shapes
  trajectory_msgs
  message_generation
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
 DIRECTORY srv
 FILES
 ReparametrizePlan.srv
 ReparametrizePlanPy.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs moveit_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS actionlib_msgs roscpp rospy std_msgs moveit_msgs trajectory_msgs message_runtime moveit_ros_planning_interface
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(plan_reparametrizer src/continue_plan.cpp)
target_link_libraries(plan_reparametrizer ${catkin_LIBRARIES})
add_dependencies(plan_reparametrizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  scripts/test_continue_plan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/reparametrize.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
