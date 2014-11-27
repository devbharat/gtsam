# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/inference

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/inference" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/inference-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/EliminationTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/BayesTreeCliqueBase.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/ClusterTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/VariableIndex.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/BayesNet.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/Conditional.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/ISAM.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/graph.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/JunctionTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/BayesTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/BayesTreeCliqueBase-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/inferenceExceptions.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/Key.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/FactorGraph.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/Conditional-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/EliminateableFactorGraph.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/VariableSlots.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/FactorGraph-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/VariableIndex-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/EliminationTree-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/ISAM-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/Symbol.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/graph-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/BayesNet-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/JunctionTree-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/Ordering.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/LabeledSymbol.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/Factor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/ClusterTree-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/EliminateableFactorGraph-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/inference/BayesTree-inst.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/inference/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

