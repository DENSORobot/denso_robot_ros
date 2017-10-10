cmake_minimum_required(VERSION 2.8.3)
project(vs060_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# other files
foreach(dir launch config worlds)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach(dir)
