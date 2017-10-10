cmake_minimum_required(VERSION 2.8.3)
project(bcap_service)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  bcap_core
)

add_message_files(
  FILES
  variant.msg
)

add_service_files(
  FILES
  bcap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bcap_service
  CATKIN_DEPENDS roscpp std_msgs message_runtime
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

add_library(bcap_service
  src/bcap_service.cpp)

add_dependencies(bcap_service
  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(bcap_service_exec
  src/bcap_service.cpp)

add_dependencies(bcap_service_exec
  ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(bcap_service
  ${catkin_LIBRARIES}
  bcap_client
  rac_string
)

target_link_libraries(bcap_service_exec
  ${catkin_LIBRARIES}
  bcap_client
  rac_string
)

#############
## Install ##
#############

install(TARGETS bcap_service bcap_service_exec
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

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
