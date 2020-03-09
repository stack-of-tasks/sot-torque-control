# Copyright 2014-2018, Andrea Del Prete, LAAS/CNRS
# Thomas Flayols, LAAS/CNRS
# Olivier Stasse, LAAS/CNRS
# Florent Forget, LAAS/CNRS
# Paul Dandignac, LAAS/CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-torque-control)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aimed at implementing torque control on different robots.")
SET(PROJECT_URL "https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python binding" OFF)
OPTION(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)

SET(CUSTOM_HEADER_DIR "sot/torque_control")
SET(DOXYGEN_USE_MATHJAX YES)
SET(CXX_DISABLE_WERROR True)
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

PKG_CONFIG_APPEND_LIBS("sot-torque-control")

SET(BOOST_COMPONENTS thread filesystem program_options unit_test_framework system regex)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python 3.0.0 REQUIRED PKG_CONFIG_REQUIRES "dynamic-graph-python >= 3.0.0")
  SET(PYTHON_LIBRARY_DIR dynamic_graph/sot/torque_control)
  SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/${PYTHON_LIBRARY_DIR})
  INCLUDE_DIRECTORIES(SYSTEM ${PYTHON_INCLUDE_DIR})
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES eigen3)
ADD_PROJECT_DEPENDENCY(dynamic-graph 3.8.0 REQUIRED PKG_CONFIG_REQUIRES "dynamic-graph >= 3.8.0")
ADD_PROJECT_DEPENDENCY(sot-core 4.8.0 REQUIRED PKG_CONFIG_REQUIRES "sot-core >= 4.8.0")
ADD_PROJECT_DEPENDENCY(pinocchio 2.3.1 REQUIRED PKG_CONFIG_REQUIRES "pinocchio >= 2.3.1")
ADD_REQUIRED_DEPENDENCY("tsid")
ADD_REQUIRED_DEPENDENCY("parametric-curves")
ADD_REQUIRED_DEPENDENCY("simple_humanoid_description")

ADD_OPTIONAL_DEPENDENCY("ddp-actuator-solver")
SET(SOTTORQUECONTROL_LIB_NAME ${PROJECT_NAME})
SET(LIBRARY_NAME ${SOTTORQUECONTROL_LIB_NAME})

SET(${LIBRARY_NAME}_HEADERS
  include/sot/torque_control/joint-torque-controller.hh
  include/sot/torque_control/base-estimator.hh
  include/sot/torque_control/numerical-difference.hh
  include/sot/torque_control/motor-model.hh
  include/sot/torque_control/joint-trajectory-generator.hh
  include/sot/torque_control/se3-trajectory-generator.hh
  include/sot/torque_control/free-flyer-locator.hh
  include/sot/torque_control/inverse-dynamics-balance-controller.hh
  include/sot/torque_control/position-controller.hh
  include/sot/torque_control/control-manager.hh
  include/sot/torque_control/current-controller.hh
  include/sot/torque_control/commands-helper.hh
  include/sot/torque_control/device-torque-ctrl.hh
  include/sot/torque_control/trace-player.hh
  include/sot/torque_control/torque-offset-estimator.hh
  include/sot/torque_control/imu_offset_compensation.hh
  include/sot/torque_control/admittance-controller.hh
  include/sot/torque_control/utils/trajectory-generators.hh
  include/sot/torque_control/utils/lin-estimator.hh
  include/sot/torque_control/utils/poly-estimator.hh
  include/sot/torque_control/utils/quad-estimator.hh
  include/sot/torque_control/utils/Stdafx.hh
  include/sot/torque_control/utils/vector-conversions.hh
  )

SET(${LIBRARY_NAME}_SOURCES ${${LIBRARY_NAME}_HEADERS}
    src/trajectory-generators.cpp
    src/lin-estimator.cpp
    src/poly-estimator.cpp
    src/quad-estimator.cpp
    src/motor-model.cpp
)


ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dynamic-graph::dynamic-graph)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} sot-core::sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} tsid)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} parametric-curves)

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python/${PYTHON_LIBRARY_DIR})
ENDIF(BUILD_PYTHON_INTERFACE)

IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${LIBRARY_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

IF(BUILD_PYTHON_INTERFACE)
  INSTALL(FILES python/dynamic_graph/sot/torque_control/__init__.py
    python/dynamic_graph/sot/torque_control/main.py
    python/dynamic_graph/sot/torque_control/create_entities_utils.py
    python/dynamic_graph/sot/torque_control/main_sim.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control)

  INSTALL(FILES
    python/dynamic_graph/sot/torque_control/identification/__init__.py
    python/dynamic_graph/sot/torque_control/identification/identification_utils.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/identification)

  INSTALL(FILES python/dynamic_graph/sot/torque_control/utils/__init__.py
    python/dynamic_graph/sot/torque_control/utils/plot_utils.py
    python/dynamic_graph/sot/torque_control/utils/sot_utils.py
    python/dynamic_graph/sot/torque_control/utils/filter_utils.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/utils)

  INSTALL(FILES python/dynamic_graph/sot/torque_control/tests/__init__.py
    python/dynamic_graph/sot/torque_control/tests/test_control_manager.py
    python/dynamic_graph/sot/torque_control/tests/test_balance_ctrl_openhrp.py
    python/dynamic_graph/sot/torque_control/tests/test_velocity_filters.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/tests)

ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)

# *****************************
# PYTHON interface            *
# *****************************
IF(BUILD_PYTHON_INTERFACE)

  IF(TALOS_DATA_FOUND)
    FOREACH(py_filename test_torque_offset_estimator)
      CONFIGURE_FILE(
        ${PROJECT_SOURCE_DIR}/python/dynamic_graph/sot/torque_control/tests/${py_filename}.py.in
        ${PROJECT_BINARY_DIR}/python/dynamic_graph/sot/torque_control/tests/${py_filename}.py
        )

      INSTALL(FILES
        ${PROJECT_BINARY_DIR}/python/dynamic_graph/sot/torque_control/tests/${py_filename}.py
        DESTINATION
        ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/tests)


    ENDFOREACH(py_filename)
  ENDIF(TALOS_DATA_FOUND)

  IF(SIMPLE_HUMANOID_DESCRIPTION_FOUND)
    FOREACH(py_filename robot_data_test)
      CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/python/dynamic_graph/sot/torque_control/tests/${py_filename}.py.in
                     ${PROJECT_BINARY_DIR}/python/dynamic_graph/sot/torque_control/tests/${py_filename}.py
        )
      INSTALL(FILES ${PROJECT_BINARY_DIR}/python/dynamic_graph/sot/torque_control/tests/${py_filename}.py
              DESTINATION
                ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/tests)
    ENDFOREACH(py_filename)
  ENDIF(SIMPLE_HUMANOID_DESCRIPTION_FOUND)
ENDIF(BUILD_PYTHON_INTERFACE)


IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  ADD_SUBDIRECTORY(include/${CUSTOM_HEADER_DIR})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)
