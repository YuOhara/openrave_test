cmake_minimum_required(VERSION 2.8.3)
project(openrave_test)
find_package(catkin REQUIRED
  roscpp
  rospy
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  sensor_msgs
  )

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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ply_clipper src/ply_clipper.cpp)
target_link_libraries(ply_clipper ${catkin_LIBRARIES})