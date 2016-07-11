cmake_minimum_required(VERSION 2.8.3)
project(openrave_test)
find_package(catkin REQUIRED message_generation)

add_service_files(FILES
  GraspAssess.srv
  )


catkin_package()