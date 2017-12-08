cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_core)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bcap_core
  bcap_service
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  ExJoints.msg
  Joints.msg
  PoseData.msg
)

add_action_files(
  FILES
  DriveString.action
  DriveValue.action
  MoveString.action
  MoveValue.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES denso_robot_core
  CATKIN_DEPENDS actionlib
    actionlib_msgs bcap_service
    message_runtime roscpp
    std_msgs
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

add_library(denso_robot_core
  src/denso_base.cpp
  src/denso_controller.cpp
  src/denso_controller_rc8.cpp
  src/denso_robot.cpp
  src/denso_robot_core.cpp
  src/denso_robot_rc8.cpp
  src/denso_task.cpp
  src/denso_variable.cpp
  src/tinyxml2.cpp
)

add_dependencies(denso_robot_core
  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(denso_robot_core_exec
  src/denso_base.cpp
  src/denso_controller.cpp
  src/denso_controller_rc8.cpp
  src/denso_robot.cpp
  src/denso_robot_core.cpp
  src/denso_robot_rc8.cpp
  src/denso_task.cpp
  src/denso_variable.cpp
  src/tinyxml2.cpp
)

add_dependencies(denso_robot_core_exec
  ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(denso_robot_core
  ${catkin_LIBRARIES}
  ${bcap_service_LIBRARIES}
)

target_link_libraries(denso_robot_core_exec
  ${catkin_LIBRARIES}
  ${bcap_service_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS denso_robot_core denso_robot_core_exec
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# other files
foreach(dir launch config)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
