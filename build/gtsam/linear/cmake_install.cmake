# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/linear

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/linear" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianDensity.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianConditional-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianJunctionTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/Errors.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianBayesTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/NoiseModel.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/KalmanFilter.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/SubgraphSolver.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/JacobianFactor-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/iterative-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/SubgraphPreconditioner.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/Sampler.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianISAM.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/JacobianFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/linearExceptions.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianFactorGraph.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/HessianFactor.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/linearAlgorithms-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/VectorValues.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianBayesTree-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/ConjugateGradientSolver.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianEliminationTree.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/HessianFactor-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianBayesNet.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/GaussianConditional.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/IterativeSolver.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/Preconditioner.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/iterative.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/linear/PCGSolver.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/linear/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

