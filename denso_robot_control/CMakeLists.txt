cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_control)

find_package(catkin REQUIRED COMPONENTS
  bcap_core
  bcap_service
  controller_manager
  denso_robot_core
  hardware_interface
  joint_limits_interface
  roscpp
  std_msgs
  transmission_interface
)

catkin_package(
  CATKIN_DEPENDS bcap_core bcap_service controller_manager
    denso_robot_core hardware_interface
    joint_limits_interface roscpp std_msgs
    transmission_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/denso_robot_hw.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${denso_robot_core_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
