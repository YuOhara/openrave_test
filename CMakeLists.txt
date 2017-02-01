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
  rospack
  cv_bridge
  )
find_package(OpenMP)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
if("${OpenCV_VERSION}" VERSION_GREATER "2.9.9")  # >= 3.0.0
  add_executable(second_grasp_finder src/second_grasp_finder.cpp)
  target_link_libraries(second_grasp_finder ${catkin_LIBRARIES})
endif()

add_executable(test_opencv src/test_opencv.cpp)
target_link_libraries(test_opencv ${catkin_LIBRARIES})

set(HRPSYS_ROS_BRIDGE_TUTORIALS_DIRECTORY ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX})
set(OPENRAVE_TEST_DIRECTORY ${PROJECT_SOURCE_DIR})

configure_file(${PROJECT_SOURCE_DIR}/scripts/hrp_hand_r.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/hrp_hand_r.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/hrp_hand_l.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/hrp_hand_l.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/hand_and_world.env.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/hand_and_world.env.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/kinfu_mesh.kinbody.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/kinfu_mesh.kinbody.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/kinfu_mesh0.kinbody.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/kinfu_mesh0.kinbody.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/kinfu_mesh_estimated.kinbody.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/kinfu_mesh_estimated.kinbody.xml)
configure_file(${PROJECT_SOURCE_DIR}/scripts/hrp.env.xml.in ${PROJECT_SOURCE_DIR}/scripts/config/hrp.env.xml)
