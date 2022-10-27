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

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
