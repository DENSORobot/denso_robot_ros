cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_core_test)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  bcap_core
  denso_robot_core
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(denso_robot_core_test
  src/denso_robot_core_test.cpp)

target_link_libraries(denso_robot_core_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS denso_robot_core_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
