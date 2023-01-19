cmake_minimum_required(VERSION 3.8)
project(denso_robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bcap_core REQUIRED)
find_package(bcap_service REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/denso_base.cpp
  src/denso_controller.cpp
  src/denso_controller_rc8.cpp
  src/denso_controller_rc8_cobotta.cpp
  src/denso_controller_rc9.cpp
  src/denso_robot.cpp
  src/${PROJECT_NAME}.cpp
  src/denso_robot_rc8.cpp
  src/denso_robot_rc8_cobotta.cpp
  src/denso_robot_rc9.cpp
  src/denso_task.cpp
  src/denso_variable.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  bcap_core
  bcap_service
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
