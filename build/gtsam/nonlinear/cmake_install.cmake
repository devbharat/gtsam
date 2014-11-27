# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/WhiteNoiseFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/Values.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/LinearContainerFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/ExtendedKalmanFilter-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearFactorGraph.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/Marginals.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/ISAM2-impl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/ExtendedKalmanFilter.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearEquality.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/GaussNewtonOptimizer.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/ISAM2.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearOptimizerParams.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearISAM.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/Symbol.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/DoglegOptimizer.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/ISAM2-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/DoglegOptimizerImpl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/nonlinearExceptions.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/Values-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/nonlinear/NonlinearOptimizer.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/nonlinear/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

