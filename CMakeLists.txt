cmake_minimum_required(VERSION 2.8.3)
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
option  (_USE_LINUX_API      "Use Linux API"   ON)
option  (_DN_USE_VARIANT_API "Use VARIANT API" ON)
option  (_DN_USE_BSTR_API    "Use BSTR API" ON)

message (STATUS "_USE_LINUX_API=${_USE_LINUX_API}")
message (STATUS "_DN_USE_VARIANT_API=${_DN_USE_VARIANT_API}")
message (STATUS "_DN_USE_BSTR_API=${_DN_USE_BSTR_API}")

# Add definitions
if(${_USE_LINUX_API})
  add_definitions(-D_USE_LINUX_API)
endif(${_USE_LINUX_API})

if(${_DN_USE_VARIANT_API})
  add_definitions(-D_DN_USE_VARIANT_API=1)
else(${_DN_USE_VARIANT_API})
  add_definitions(-D_DN_USE_VARIANT_API=0)
endif(${_DN_USE_VARIANT_API})

if(${_DN_USE_BSTR_API})
  add_definitions(-D_DN_USE_BSTR_API=1)
else(${_DN_USE_BSTR_API})
  add_definitions(-D_DN_USE_BSTR_API=0)
endif(${_DN_USE_BSTR_API})

include_directories(
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/bCAPClient
  include/${PROJECT_NAME}/bCAPServer
  include/${PROJECT_NAME}/TPComm
  include/${PROJECT_NAME}/RACString
)

add_library(bcap_client
  src/bCAPClient/bcap_client.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_udp.c
  src/dn_com.c
  src/dn_thread.c
  src/bcap_common.c
)

target_link_libraries(bcap_client
  pthread
)

add_library(bcap_server
  src/bCAPServer/bcap_server.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_udp.c
  src/dn_com.c
  src/dn_thread.c
  src/bcap_common.c
)

target_link_libraries(bcap_server
  pthread
)

add_library(tpcomm
  src/TPComm/dn_tpcomm.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_udp.c
  src/dn_com.c
  src/dn_thread.c
  src/dn_robotalk.c
)

target_link_libraries(tpcomm
  pthread
)

add_library(rac_string
  src/RACString/rac_string.c
  src/dn_common.c
)

#############
## Install ##
#############

# libraries
install(TARGETS bcap_client bcap_server tpcomm rac_string
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "bcap_core" EXCLUDE
  PATTERN ".svn" EXCLUDE
)
