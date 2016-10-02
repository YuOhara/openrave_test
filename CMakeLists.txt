cmake_minimum_required(VERSION 2.8.3)
project(openrave_test)
find_package(catkin REQUIRED
  roscpp
  rospy
  geometry_msgs
  message_generation
  message_filters
  pcl_conversions
  pcl_ros
  sensor_msgs
  image_geometry
  tf_conversions
  eigen_conversions
  hrpsys_ros_bridge_tutorials
  )

add_message_files(FILES
  RaveGraspArray.msg
  )
add_service_files(FILES
  GraspAssess.srv
  )
add_service_files(FILES
  SecondGrasp.srv
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
add_executable(second_grasp_finder src/second_grasp_finder.cpp)
target_link_libraries(second_grasp_finder ${catkin_LIBRARIES})

set(HRPSYS_ROS_BRIDGE_TUTORIALS_DIRECTORY ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX})

configure_file(${PROJECT_SOURCE_DIR}/scripts/hrp_hand_r.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/hrp_hand_r.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/hrp_hand_l.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/hrp_hand_l.xml)