cmake_minimum_required(VERSION 2.8.3)
project(bcap_service)

find_package(catkin REQUIRED COMPONENTS
  bcap_core
  message_generation
  roscpp
  std_msgs
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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS bcap_core message_runtime roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(${PROJECT_NAME}_exec
  src/${PROJECT_NAME}.cpp)

add_dependencies(${PROJECT_NAME}_exec
  ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${bcap_core_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_exec
  ${catkin_LIBRARIES}
  ${bcap_core_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_exec
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
