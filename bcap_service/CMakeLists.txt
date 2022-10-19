cmake_minimum_required(VERSION 3.8)
project(bcap_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bcap_core REQUIRED)

set(msg_files
  "msg/Variant.msg"
)
set(srv_files
  "srv/Bcap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
# ament_export_dependencies(rosidl_default_runtime)


###########
## Build ##
###########

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}.cpp)

# add_dependencies(${PROJECT_NAME}
#   ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(${PROJECT_NAME}_exec
#   src/${PROJECT_NAME}.cpp)

# add_dependencies(${PROJECT_NAME}_exec
#   ${PROJECT_NAME}_generate_messages_cpp)

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${bcap_core_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}_exec
#   ${catkin_LIBRARIES}
#   ${bcap_core_LIBRARIES}
# )

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_exec
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# # launch
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
