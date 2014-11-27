# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/geometry" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Pose3.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Point3.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Cal3_S2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Rot2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/concepts.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/EssentialMatrix.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Cal3Bundler.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Cal3DS2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Point2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Rot3.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/TriangulationFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Cal3DS2_Base.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Unit3.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Cal3_S2Stereo.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Cal3Unified.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/StereoPoint2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/CalibratedCamera.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/triangulation.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/SimpleCamera.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/Pose2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/PinholeCamera.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/geometry/StereoCamera.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/geometry/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

