cmake_minimum_required(VERSION 2.8.3)
project(openrave_test)
find_package(catkin REQUIRED
  roscpp
  rospy
  geometry_msgs
  message_generation)

add_service_files(FILES
  GraspAssess.srv
  )

generate_messages (
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )