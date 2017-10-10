cmake_minimum_required(VERSION 2.8.3)
project(denso_launch)

find_package(catkin REQUIRED)

catkin_package(
  DEPENDS
  CATKIN-DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

#############
## Install ##
#############

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
