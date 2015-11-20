cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ymcl)

set(PROJECT_VERSION 1.0.0 CACHE STRING "ymcl version")
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_REVISION 2)
set(PROJECT_VERSION_CANDIDATE 0)
# DISSECT_VERSION()
set(PROJECT_DESCRIPTION "Monte Calro Localization Library")
set(PROJECT_VENDOR "Sugar Sweet Robotics")
set(PROJECT_AUTHOR "Sugar Sweet Robotics")
set(PROJECT_AUTHOR_SHORT "Sugar Sweet Robotics")


set(BIN_INSTALL_DIR "bin")
set(LIB_INSTALL_DIR "lib")
set(MAN_INSTALL_DIR "share/man/man3")


option(STATIC_LIBS "Build static libraries" OFF)

if(STATIC_LIBS)
    set(LIB_TYPE STATIC)
else(STATIC_LIBS)
    set(LIB_TYPE SHARED)
endif(STATIC_LIBS)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   # Mac OS X specific code
   SET(CMAKE_CXX_COMPILER "g++")
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include)
set(hdrs)
foreach(hdr ${inc_headers})
  set(hdrs ${hdrs}  "${CMAKE_CURRENT_SOURCE_DIR}/include/${hdr}")
endforeach(hdr)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)


install(
   EXPORT ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_DIR}
)
