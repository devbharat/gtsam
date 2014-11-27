# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/discrete" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteJunctionTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/Assignment.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteKey.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteEliminationTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteBayesNet.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/Signature.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteBayesTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DecisionTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/Potentials.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteMarginals.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/AlgebraicDecisionTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DecisionTreeFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteConditional.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DecisionTree-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/discrete/DiscreteFactorGraph.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/discrete/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

