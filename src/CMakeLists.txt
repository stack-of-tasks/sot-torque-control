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

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

#define DEBUG=2 if we're building in debug mode (what for?)
IF("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

#This project will create many plugins as shared libraries, listed here
SET(plugins
  joint-torque-controller
  force-torque-estimator
  numerical-difference
  joint-trajectory-generator
  nd-trajectory-generator
  se3-trajectory-generator
  madgwickahrs
  inverse-dynamics-controller
  inverse-dynamics-balance-controller
  position-controller
  admittance-controller
  control-manager
  hrp2-device-pos-ctrl
  #hrp2-device-torque-ctrl
  free-flyer-locator
  )

#set(ADDITIONAL_feature-task_LIBS feature-generic task)


#Plugins compilation, link, and installation
#Compiles a plugin. The plugin library is ${LIBRARY_NAME}
FOREACH(plugin ${plugins})
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

  # build python submodule
  # (replace minus with underscore to make Python happy)
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/torque_control/${PYTHON_LIBRARY_NAME}"
    ${LIBRARY_NAME}
    sot-torque-control-${PYTHON_LIBRARY_NAME}-wrap
    )
  # Install plugins
  INSTALL(TARGETS ${LIBRARY_NAME}
    DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
ENDFOREACH(plugin)

# Bindings Python
DYNAMIC_GRAPH_PYTHON_MODULE("sot/torque_control" ${SOTTORQUECONTROL_LIB_NAME} wrap)

# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/torque_control/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control
)
