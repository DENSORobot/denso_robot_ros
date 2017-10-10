cmake_minimum_required(VERSION 2.8.3)
project(bcap_service_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bcap_service
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bcap_service_test
  src/bcap_service_test.cpp)

target_link_libraries(bcap_service_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS bcap_service_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
