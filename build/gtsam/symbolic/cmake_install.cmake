# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/symbolic" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicBayesTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicBayesNet.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicJunctionTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicFactorGraph.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicEliminationTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicISAM.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicConditional.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/symbolic/SymbolicFactor-inst.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/symbolic/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

