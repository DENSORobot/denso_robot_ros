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
  CATKIN_DEPENDS bcap_service controller_manager
    denso_robot_core hardware_interface
    joint_limits_interface roscpp std_msgs
    transmission_interface
  DEPENDS bcap_core
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
)

add_executable(denso_robot_control
  src/denso_robot_control.cpp
  src/denso_robot_hw.cpp
)

target_link_libraries(denso_robot_control
  ${catkin_LIBRARIES}
  ${denso_robot_core_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS denso_robot_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
