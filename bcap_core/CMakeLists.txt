cmake_minimum_required(VERSION 2.8.11)
project(bcap_core)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bcap_client bcap_server tpcomm rac_string
)

###########
## Build ##
###########

# Options
option  (${PROJECT_NAME}_USE_LINUX_API      "Use Linux API"   ON)
option  (${PROJECT_NAME}_DN_USE_VARIANT_API "Use VARIANT API" ON)
option  (${PROJECT_NAME}_DN_USE_BSTR_API    "Use BSTR API"    ON)

message (STATUS "_USE_LINUX_API=${${PROJECT_NAME}_USE_LINUX_API}")
message (STATUS "_DN_USE_VARIANT_API=${${PROJECT_NAME}_DN_USE_VARIANT_API}")
message (STATUS "_DN_USE_BSTR_API=${${PROJECT_NAME}_DN_USE_BSTR_API}")

# Add definitions
if(${${PROJECT_NAME}_USE_LINUX_API})
  add_definitions(-D_USE_LINUX_API)
endif(${${PROJECT_NAME}_USE_LINUX_API})

if(${${PROJECT_NAME}_DN_USE_VARIANT_API})
  add_definitions(-D_DN_USE_VARIANT_API=1)
else(${${PROJECT_NAME}_DN_USE_VARIANT_API})
  add_definitions(-D_DN_USE_VARIANT_API=0)
endif(${${PROJECT_NAME}_DN_USE_VARIANT_API})

if(${${PROJECT_NAME}_DN_USE_BSTR_API})
  add_definitions(-D_DN_USE_BSTR_API=1)
else(${${PROJECT_NAME}_DN_USE_BSTR_API})
  add_definitions(-D_DN_USE_BSTR_API=0)
endif(${${PROJECT_NAME}_DN_USE_BSTR_API})

include_directories(
  include
)

add_library(bcap_client
  src/bCAPClient/bcap_client.c
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c
)

target_include_directories(bcap_client PUBLIC
  include/bCAPClient
)

target_link_libraries(bcap_client
  pthread
)

add_library(bcap_server
  src/bCAPServer/bcap_server.c
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c
)

target_include_directories(bcap_server PUBLIC
  include/bCAPServer
)

target_link_libraries(bcap_server
  pthread
)

add_library(rac_string
  src/RACString/rac_string.c
  src/dn_common.c
)

target_include_directories(rac_string PUBLIC
  include/RACString
)

add_library(tpcomm
  src/TPComm/dn_tpcomm.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_robotalk.c
  src/dn_socket.c
  src/dn_thread.c
  src/dn_udp.c
)

target_include_directories(tpcomm PUBLIC
  include/TPComm
)

target_link_libraries(tpcomm
  pthread
)

#############
## Install ##
#############

# libraries
install(TARGETS bcap_client bcap_server rac_string tpcomm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "${PROJECT_NAME}" EXCLUDE
  PATTERN ".svn" EXCLUDE
)
