cmake_minimum_required(VERSION 3.0.2)
project(theremini_player)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()
include_directories()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/random
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
