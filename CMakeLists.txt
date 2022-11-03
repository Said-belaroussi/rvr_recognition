cmake_minimum_required(VERSION 3.0.2)
project(rvr_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
    FILES
    PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   nodes/color_detector/color_detector
   nodes/color_detector/color_translator
   nodes/aruco_detector/aruco_translator
   nodes/nn_detector/nn_detector
   nodes/nn_detector/nn_translator
   nodes/rvr_localisator
   nodes/rvr_localisator_viewer
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(
  DIRECTORY nodes/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
