# Copyright 2014-2020 Andrea Del Prete, LAAS-CNRS Thomas Flayols, LAAS-CNRS
# Olivier Stasse, LAAS-CNRS Florent Forget, LAAS-CNRS Paul Dandignac, LAAS-CNRS
# Noëlie Ramuzat, LAAS-CNRS Guilhem Saurel, LAAS-CNRS
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-torque-control)
set(PROJECT_DESCRIPTION
    "Collection of dynamic-graph entities aimed at implementing torque control on different robots."
)
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "sot/torque_control")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(INITIALIZE_WITH_NAN)
  message(STATUS "Initialize with NaN all the Eigen entries.")
  add_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
endif(INITIALIZE_WITH_NAN)

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
  string(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency(dynamic-graph 4.4.0 REQUIRED)
add_project_dependency(sot-core REQUIRED)
add_project_dependency(tsid REQUIRED)
add_project_dependency(ddp-actuator-solver REQUIRED)
add_project_dependency(parametric-curves REQUIRED)

if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  add_project_dependency(example-robot-data 3.8.0 REQUIRED)
endif(BUILD_TESTING)

if(Boost_VERSION GREATER 107299)
  # Silence a warning about a deprecated use of boost bind by boost python at
  # least fo boost 1.73 to 1.75
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

# Main Library
set(${PROJECT_NAME}_HEADERS
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
    include/${CUSTOM_HEADER_DIR}/utils/vector-conversions.hh)

set(${PROJECT_NAME}_SOURCES
    src/trajectory-generators.cpp src/lin-estimator.cpp src/poly-estimator.cpp
    src/quad-estimator.cpp src/motor-model.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC tsid::tsid sot-core::sot-core
                         ddp-actuator-solver::ddp-actuator-solver)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
add_subdirectory(tests)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
