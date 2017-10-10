cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  joint_limits_interface
  transmission_interface
  controller_manager
  bcap_core
  bcap_service
  denso_robot_core
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
    hardware_interface joint_limits_interface
    transmission_interface controller_manager
    bcap_service denso_robot_core
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
  denso_robot_core
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
