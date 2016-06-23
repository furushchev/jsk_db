cmake_minimum_required(VERSION 2.8.3)
project(jsk_db_msgs)

find_package(catkin REQUIRED
  message_generation
  geometry_msgs
  std_msgs
  )

add_message_files(
  FILES
  KeyValue.msg
  ObjectAnnotation.msg
  Spot.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
)
