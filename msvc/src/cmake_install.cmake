# Install script for directory: D:/Development/op3d_active/osgearth-2.8/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/OSGEARTH")
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

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarth/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthAnnotation/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthFeatures/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthSymbology/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthUtil/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthQt/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthSplat/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthSilverLining/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthTriton/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/osgEarthDrivers/cmake_install.cmake")
  INCLUDE("D:/Development/op3d_active/osgearth-2.8/msvc/src/applications/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

