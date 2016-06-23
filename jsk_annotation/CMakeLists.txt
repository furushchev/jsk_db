cmake_minimum_required(VERSION 2.8.3)
project(jsk_annotation)

find_package(catkin REQUIRED COMPONENTS
  jsk_pcl_ros
  jsk_perception
  mongodb_store
  roscpp
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ObjectAnnotation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS jsk_pcl_ros jsk_perception mongodb_store std_msgs message_runtime
)

