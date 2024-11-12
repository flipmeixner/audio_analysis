cmake_minimum_required(VERSION 3.0.2)
project(audio_analysis)


find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/audio_stream_node.py
  scripts/audio_processor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


