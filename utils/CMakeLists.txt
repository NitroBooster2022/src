cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   IMU.msg
   localisation.msg
   Sign.msg
   Lane.msg
   encoder.msg
   ImgInfo.msg
 )
 
## Generate services in the 'srv' folder
 add_service_files(
  FILES
  Localization.srv
  nav.srv
  get_direction.srv
  dotted.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
