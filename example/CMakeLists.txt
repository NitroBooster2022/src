cmake_minimum_required(VERSION 3.0.2)
project(example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  utils
)
add_message_files(
  FILES
  Lane.msg
  Sign.msg
)
add_service_files(
  FILES
  Localization.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs#   tf2_geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES example
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/lane.py
  scripts/sign.py
  scripts/localization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

