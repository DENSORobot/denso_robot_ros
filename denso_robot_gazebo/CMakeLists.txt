cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# other files
foreach(dir launch worlds)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
