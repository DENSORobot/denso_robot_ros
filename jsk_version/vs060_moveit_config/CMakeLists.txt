cmake_minimum_required(VERSION 2.8.3)
project(vs060_moveit_config)

find_package(catkin REQUIRED
  roslang
  roscpp
  moveit_commander
  moveit_ros_planning
)

catkin_package(
  CATKIN_DEPENDS roslang roscpp moveit_commander moveit_ros_planning
)

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(publish_scene_from_text
  src/publish_scene_from_text.cpp)

target_link_libraries(publish_scene_from_text
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS publish_scene_from_text
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# scene
install(DIRECTORY scene
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# script
file(GLOB prgs RELATIVE ${PROJECT_SOURCE_DIR} "scripts/*.py")
foreach(prg ${prgs})
   catkin_install_python(PROGRAMS ${prg}
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(prg)
