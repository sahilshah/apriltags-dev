# Install script for directory: /Users/sahilshah/Desktop/developer/ceres/apriltags-dev/example

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/build")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/bin/cerestags_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cerestags_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cerestags_demo")
    execute_process(COMMAND /usr/bin/install_name_tool
      -delete_rpath "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cerestags_demo")
    execute_process(COMMAND /usr/bin/install_name_tool
      -delete_rpath "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/build/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cerestags_demo")
    execute_process(COMMAND /usr/bin/install_name_tool
      -add_rpath "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/build/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cerestags_demo")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cerestags_demo")
    endif()
  endif()
endif()

