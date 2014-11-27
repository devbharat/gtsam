# Install script for directory: /home/bharat/gtsam/gtsam-3.2.0/gtsam/base

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Vector.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Group.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/lieProxies.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/LieMatrix.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/debug.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Value.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/FastDefaultAllocator.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/SymmetricBlockMatrix.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/FastSet.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/ConcurrentMap.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/cholesky.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Lie-inl.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/numericalDerivative.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/SymmetricBlockMatrixBlockExpr.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Manifold.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/serializationTestHelpers.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Matrix.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/FastList.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/serialization.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/TestableAssertions.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/treeTraversal-inst.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/DerivedValue.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/LieScalar.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Testable.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/Lie.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/VerticalBlockMatrix.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/types.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/timing.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/FastVector.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/DSFVector.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/FastMap.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/LieVector.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/base/treeTraversal" TYPE FILE FILES
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/treeTraversal/statistics.h"
    "/home/bharat/gtsam/gtsam-3.2.0/gtsam/base/treeTraversal/parallelTraversalTasks.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/bharat/gtsam/gtsam-3.2.0/build/gtsam/base/tests/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

