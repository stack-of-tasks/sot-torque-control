# Copyright 2014-2020
# Andrea Del Prete, LAAS-CNRS
# Thomas Flayols, LAAS-CNRS
# Olivier Stasse, LAAS-CNRS
# Florent Forget, LAAS-CNRS
# Paul Dandignac, LAAS-CNRS
# Noëlie Ramuzat, LAAS-CNRS
# Guilhem Saurel, LAAS-CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-torque-control)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aimed at implementing torque control on different robots.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
OPTION(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)

# Project configuration
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
SET(CUSTOM_HEADER_DIR "sot/torque_control")
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

# Project dependencies
ADD_PROJECT_DEPENDENCY(sot-core REQUIRED)
ADD_PROJECT_DEPENDENCY(tsid REQUIRED)
ADD_PROJECT_DEPENDENCY(ddp-actuator-solver REQUIRED)
ADD_PROJECT_DEPENDENCY(parametric-curves REQUIRED)

IF(BUILD_TESTING)
  FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework)
  ADD_PROJECT_DEPENDENCY(example-robot-data 3.8.0 REQUIRED)
ENDIF(BUILD_TESTING)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SEARCH_FOR_BOOST_PYTHON(REQUIRED)
  STRING(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python 4.0.0 REQUIRED)

  IF(Boost_VERSION GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost python
    # at least fo boost 1.73 to 1.75
    ADD_DEFINITIONS(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  ENDIF()
ENDIF(BUILD_PYTHON_INTERFACE)

# Main Library
SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/joint-torque-controller.hh
  include/${CUSTOM_HEADER_DIR}/base-estimator.hh
  include/${CUSTOM_HEADER_DIR}/numerical-difference.hh
  include/${CUSTOM_HEADER_DIR}/motor-model.hh
  include/${CUSTOM_HEADER_DIR}/joint-trajectory-generator.hh
  include/${CUSTOM_HEADER_DIR}/se3-trajectory-generator.hh
  include/${CUSTOM_HEADER_DIR}/free-flyer-locator.hh
  include/${CUSTOM_HEADER_DIR}/inverse-dynamics-balance-controller.hh
  include/${CUSTOM_HEADER_DIR}/position-controller.hh
  include/${CUSTOM_HEADER_DIR}/control-manager.hh
  include/${CUSTOM_HEADER_DIR}/current-controller.hh
  include/${CUSTOM_HEADER_DIR}/commands-helper.hh
  include/${CUSTOM_HEADER_DIR}/device-torque-ctrl.hh
  include/${CUSTOM_HEADER_DIR}/trace-player.hh
  include/${CUSTOM_HEADER_DIR}/torque-offset-estimator.hh
  include/${CUSTOM_HEADER_DIR}/imu_offset_compensation.hh
  include/${CUSTOM_HEADER_DIR}/admittance-controller.hh
  include/${CUSTOM_HEADER_DIR}/utils/trajectory-generators.hh
  include/${CUSTOM_HEADER_DIR}/utils/lin-estimator.hh
  include/${CUSTOM_HEADER_DIR}/utils/poly-estimator.hh
  include/${CUSTOM_HEADER_DIR}/utils/quad-estimator.hh
  include/${CUSTOM_HEADER_DIR}/utils/Stdafx.hh
  include/${CUSTOM_HEADER_DIR}/utils/vector-conversions.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/trajectory-generators.cpp
  src/lin-estimator.cpp
  src/poly-estimator.cpp
  src/quad-estimator.cpp
  src/motor-model.cpp
  )


ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
  tsid::tsid sot-core::sot-core ddp-actuator-solver::ddp-actuator-solver)

IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

ADD_SUBDIRECTORY(src)
IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)
ADD_SUBDIRECTORY(tests)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
