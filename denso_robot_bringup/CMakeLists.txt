cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_bringup)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# script
file(GLOB prgs RELATIVE ${PROJECT_SOURCE_DIR} "scripts/*.py")
foreach(prg ${prgs})
   catkin_install_python(PROGRAMS ${prg}
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()
