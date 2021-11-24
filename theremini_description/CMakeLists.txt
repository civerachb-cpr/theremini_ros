cmake_minimum_required(VERSION 3.0.2)
project(theremini_description)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
