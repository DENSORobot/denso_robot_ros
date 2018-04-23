cmake_minimum_required(VERSION 2.8.11)
project(bcap_core)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
		"Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
		FORCE)
endif(NOT CMAKE_BUILD_TYPE)

###########
## Build ##
###########

# Options
option  (_DN_USE_VARIANT_API "Use VARIANT API" ON)
option  (_DN_USE_BSTR_API    "Use BSTR API"    ON)

# Windos
if(WIN32)
  set(_STATIC_LIBRARY_POSTFIX "_static" CACHE STRING "Suffix of static lib")
  
  # Define debug suffix
  if (NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Suffix of debug")
  endif(NOT DEFINED CMAKE_DEBUG_POSTFIX)
  
  # Check target architecture
  if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    # 64bit
    set(_TARGET_ARCHITECTURE "x64/" CACHE INTERNAL "Target Architecture")
  elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    # 32bit
    set(_TARGET_ARCHITECTURE "x86/" CACHE INTERNAL "Target Architecture")
  else(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    ## unknown
    set(_TARGET_ARCHITECTURE "" CACHE INTERNAL "")
  endif(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
else(WIN32) # Linux
  # Search catkin
  find_package(catkin QUIET)
  if(catkin_FOUND)
    option  (_USE_CATKIN "Use catkin for installation" ON)
    if(_USE_CATKIN)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES bcap_client bcap_server tpcomm rac_string
    )
    message (STATUS "_USE_CATKIN=${_USE_CATKIN}")
    endif(_USE_CATKIN)
  else(catkin_FOUND)
    message(STATUS "catkin was not found. Build without catkin.")
  endif(catkin_FOUND)

endif(WIN32)

# Use visual studio
if(MSVC)
  # Set VS version
  if("${CMAKE_GENERATOR}" MATCHES "Visual Studio ([0-9]?[0-9])")
    set(_VS_VER "v${CMAKE_MATCH_1}/" CACHE INTERNAL "Visual Studio Ver")
  else("${CMAKE_GENERATOR}" MATCHES "Visual Studio ([0-9]?[0-9])")
    set(_VS_VER "" CACHE INTERNAL "")
  endif("${CMAKE_GENERATOR}" MATCHES "Visual Studio ([0-9]?[0-9])")
else(MSVC) # Not use visual studio
  option  (_USE_LINUX_API      "Use Linux API"   ON)
  message (STATUS "_USE_LINUX_API=${_USE_LINUX_API}")
endif(MSVC)

option (BUILD_SHARED_LIBS "Build as a shared library" OFF)

message (STATUS "_DN_USE_VARIANT_API=${_DN_USE_VARIANT_API}")
message (STATUS "_DN_USE_BSTR_API=${_DN_USE_BSTR_API}")
message (STATUS "BUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}")

# Add definitions
if(_USE_LINUX_API)
  add_definitions(-D_USE_LINUX_API)
else(_USE_LINUX_API)
  add_definitions(-D_USE_WIN_API)
  add_definitions("-D_DN_EXP_RACSTR=__declspec(dllexport)")
  add_definitions("-D_DN_EXP_COMMON=__declspec(dllexport)")
endif(_USE_LINUX_API)

if(_DN_USE_VARIANT_API)
  add_definitions(-D_DN_USE_VARIANT_API=1)
else(_DN_USE_VARIANT_API)
  add_definitions(-D_DN_USE_VARIANT_API=0)
endif(_DN_USE_VARIANT_API)

if(_DN_USE_BSTR_API)
  add_definitions(-D_DN_USE_BSTR_API=1)
else(_DN_USE_BSTR_API)
  add_definitions(-D_DN_USE_BSTR_API=0)
endif(_DN_USE_BSTR_API)

include_directories(
  include
)

# Include stdint.h
if(MSVC AND "${_VS_VER}" MATCHES "v([0-9]?[0-9])")
  if("${CMAKE_MATCH_1}" LESS 10)
    include_directories(
      include/stdint
    )
  endif("${CMAKE_MATCH_1}" LESS 10)
endif(MSVC AND "${_VS_VER}" MATCHES "v([0-9]?[0-9])")


# bcap_client library
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

if(_USE_LINUX_API)
  target_link_libraries(bcap_client
    pthread
  )
endif(_USE_LINUX_API)

# bcap_server library
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

if(_USE_LINUX_API)
  target_link_libraries(bcap_server
    pthread
  )
endif(_USE_LINUX_API)

# rac_string library
add_library(rac_string
  src/RACString/rac_string.c
  src/dn_common.c
)

target_include_directories(rac_string PUBLIC
  include/RACString
)

# tpcomm library
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

if(_USE_LINUX_API)
  target_link_libraries(tpcomm
    pthread
  )
endif(_USE_LINUX_API)

#############
## Install ##
#############

if(_USE_CATKIN)
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
else(_USE_CATKIN)
  # libraries
  if(WIN32 AND NOT BUILD_SHARED_LIBS)
    set_target_properties(bcap_client bcap_server tpcomm rac_string
      PROPERTIES SUFFIX "${_STATIC_LIBRARY_POSTFIX}.lib"
    )
  endif(WIN32 AND NOT BUILD_SHARED_LIBS)

  install(TARGETS bcap_client bcap_server tpcomm rac_string
    ARCHIVE DESTINATION ${_TARGET_ARCHITECTURE}${_VS_VER}lib
    LIBRARY DESTINATION ${_TARGET_ARCHITECTURE}${_VS_VER}lib
    RUNTIME DESTINATION ${_TARGET_ARCHITECTURE}${_VS_VER}bin
    )

  # headers
  install(
    DIRECTORY include/
    DESTINATION include/orin_bcap
    FILES_MATCHING PATTERN "*.h"
    PATTERN "${PROJECT_NAME}" EXCLUDE
    PATTERN ".svn" EXCLUDE
  )

endif(_USE_CATKIN)