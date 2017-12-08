cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# other files
foreach(dir launch config)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
