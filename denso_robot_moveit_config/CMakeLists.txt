cmake_minimum_required(VERSION 3.22)
project(denso_robot_moveit_config)

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/launch_common.py)

ament_package()