# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/slam

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/slam" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/RangeFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/JacobianFactorQR.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/PoseRotationPrior.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/dataset.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/SmartFactorBase.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/EssentialMatrixConstraint.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/lago.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/SmartProjectionFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/RotateFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/BoundingConstraint.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/BetweenFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/ProjectionFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/PoseTranslationPrior.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/AntiFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/StereoFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/RegularHessianFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/GeneralSFMFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/JacobianFactorSVD.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/InitializePose3.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/EssentialMatrixFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/SmartProjectionPoseFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/BearingFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/ReferenceFrameFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/ImplicitSchurFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/BearingRangeFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/JacobianSchurFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/PriorFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/slam/JacobianFactorQ.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/slam/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

