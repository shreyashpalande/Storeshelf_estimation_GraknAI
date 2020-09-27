cmake_minimum_required(VERSION 2.8.3)
project(graknrosservice)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_generation
)

add_service_files(
  FILES
  Graknquery.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS  message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
