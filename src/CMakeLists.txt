# Copyright 2014, Andrea Del Prete, LAAS/CNRS
#
# This file is part of sot-torque-control.
# sot-torque-control is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-torque-control is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-torque-control. If not, see <http://www.gnu.org/licenses/>.

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
IF(BUILD_PYTHON_INTERFACE)
  LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})
ENDIF(BUILD_PYTHON_INTERFACE)

#define DEBUG=2 if we're building in debug mode (what for?)
IF("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

#This project will create many plugins as shared libraries, listed here
SET(plugins
  admittance-controller  
  base-estimator
  control-manager
  current-controller
  device-torque-ctrl  
  filter-differentiator  
  free-flyer-locator
  imu_offset_compensation  
  inverse-dynamics-balance-controller
  joint-torque-controller
  joint-trajectory-generator
  nd-trajectory-generator
  numerical-difference  
  position-controller  
  se3-trajectory-generator
  torque-offset-estimator
  trace-player
  )

IF(DDP_ACTUATOR_SOLVER_FOUND)
  SET(plugins ${plugins} ddp-actuator-solver )
ENDIF(DDP_ACTUATOR_SOLVER_FOUND)

#Plugins compilation, link, and installation
#Compiles a plugin. The plugin library is ${LIBRARY_NAME}
FOREACH(plugin ${plugins})

  message( " Build plugin ${plugin}")
  
  #retrieve plugin name
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)

  # only one source file per plugin
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${plugin}.cpp)

  #remove the "lib" prefix from the plugin output name
  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    PREFIX "")

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH ${DYNAMIC_GRAPH_PLUGINDIR})

  # Link with sot-torque-control library
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOTTORQUECONTROL_LIB_NAME})
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

  ADD_DEPENDENCIES(${LIBRARY_NAME} ${SOTTORQUECONTROL_LIB_NAME})

  IF(ADDITIONAL_${LIBRARY_NAME}_LIBS)
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
  ENDIF(ADDITIONAL_${LIBRARY_NAME}_LIBS)

  # Linux dynamic loading library flags
  IF(UNIX)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
  ENDIF(UNIX)

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl pthread)
  ENDIF(UNIX AND NOT APPLE)

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

  IF(DDP_ACTUATOR_SOLVER_FOUND)
    SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/qpOASES")
    FIND_PACKAGE("qpOASES" REQUIRED)
    INCLUDE_DIRECTORIES(SYSTEM ${qpOASES_INCLUDE_DIRS})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${qpOASES_LIBRARIES})
    PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} ddp-actuator-solver )
  ENDIF(DDP_ACTUATOR_SOLVER_FOUND)

  # build python submodule
  # (replace minus with underscore to make Python happy)
  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    DYNAMIC_GRAPH_PYTHON_MODULE("sot/torque_control/${PYTHON_LIBRARY_NAME}"
      ${LIBRARY_NAME}
      sot-torque-control-${PYTHON_LIBRARY_NAME}-wrap
      )
  ENDIF(BUILD_PYTHON_INTERFACE)
  IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
    # Install plugins
    INSTALL(TARGETS ${LIBRARY_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
ENDFOREACH(plugin)

# Bindings Python
IF(BUILD_PYTHON_INTERFACE)
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/torque_control" ${SOTTORQUECONTROL_LIB_NAME} wrap)

  # Install empty __init__.py files in intermediate directories.
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/torque_control/__init__.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control
  )
ENDIF(BUILD_PYTHON_INTERFACE)
