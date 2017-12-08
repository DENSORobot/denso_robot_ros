cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_descriptions)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# other files
file(GLOB descs RELATIVE ${PROJECT_SOURCE_DIR} "*_description")
foreach(dir ${descs})
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
