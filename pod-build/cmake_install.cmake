# Install script for directory: /Users/sahilshah/Desktop/developer/ceres/apriltags-dev

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/lib/libapriltags.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.a")
    execute_process(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libapriltags.a")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/AprilTags" TYPE FILE FILES
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Edge.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/FloatImage.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Gaussian.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/GLine2D.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/GLineSegment2D.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/GrayModel.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Gridder.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Homography33.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/MathUtil.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/pch.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Quad.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/RunningStats.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Segment.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag16h5.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag16h5_other.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag25h7.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag25h9.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag36h11.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag36h11_other.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/Tag36h9.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/TagDetection.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/TagDetector.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/TagFamily.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/UnionFindSimple.h"
    "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/AprilTags/XYWeight.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/lib/pkgconfig/apriltags.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/example/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

file(WRITE "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/${CMAKE_INSTALL_MANIFEST}" "")
foreach(file ${CMAKE_INSTALL_MANIFEST_FILES})
  file(APPEND "/Users/sahilshah/Desktop/developer/ceres/apriltags-dev/pod-build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
endforeach()
