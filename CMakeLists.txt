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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME sot-torque-control)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aiming at the implementation of torque control on HRP-2.")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-torque-control")

SET(CUSTOM_HEADER_DIR "sot/torque-control")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
#add_compile_options(-std=c++11) # CMake 2.8.12 or newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

PKG_CONFIG_APPEND_LIBS("sot-torque-control")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS thread filesystem program_options unit_test_framework system regex )
SEARCH_FOR_BOOST()

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >=3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2")
ADD_REQUIRED_DEPENDENCY("PinInvDyn")


SET(SOTTORQUECONTROL_LIB_NAME ${PROJECT_NAME})
SET(LIBRARY_NAME ${SOTTORQUECONTROL_LIB_NAME})

SET(${LIBRARY_NAME}_HEADERS
  include/sot/torque_control/joint-torque-controller.hh
  include/sot/torque_control/force-torque-estimator.hh
  include/sot/torque_control/numerical-difference.hh
  include/sot/torque_control/motor-model.hh
  include/sot/torque_control/joint-trajectory-generator.hh
  include/sot/torque_control/nd-trajectory-generator.hh
  include/sot/torque_control/se3-trajectory-generator.hh
  include/sot/torque_control/free-flyer-locator.hh
  include/sot/torque_control/inverse-dynamics-balance-controller.hh
  include/sot/torque_control/position-controller.hh
  include/sot/torque_control/control-manager.hh
  include/sot/torque_control/commands-helper.hh
  include/sot/torque_control/signal-helper.hh
  include/sot/torque_control/utils/logger.hh
  include/sot/torque_control/utils/trajectory-generators.hh
  include/sot/torque_control/utils/lin-estimator.hh
  include/sot/torque_control/utils/poly-estimator.hh
  include/sot/torque_control/utils/quad-estimator.hh
#  include/sot/torque_control/utils/Stdafx.hh
#  include/sot/torque_control/utils/stop-watch.hh
  include/sot/torque_control/utils/vector-conversions.hh
  )

#INSTALL(FILES ${${LIBRARY_NAME}_HEADERS}
#  DESTINATION include/sot/torque-control
#  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

SET(${LIBRARY_NAME}_SOURCES ${${LIBRARY_NAME}_HEADERS}
    src/logger.cpp
    src/trajectory-generators.cpp
    src/lin-estimator.cpp
    src/poly-estimator.cpp
    src/quad-estimator.cpp
    src/stop-watch.cpp
    src/motor-model.cpp
)

SET(${LIBRARY_NAME}_PYTHON_FILES python/*.py)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} PinInvDyn)


IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

FINDPYTHON()
INSTALL(FILES python/dynamic_graph/sot/torque_control/__init__.py
              python/dynamic_graph/sot/torque_control/main.py
              python/dynamic_graph/sot/torque_control/create_entities_utils.py
              python/dynamic_graph/sot/torque_control/main_sim.py
         DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control)
         
INSTALL(FILES python/dynamic_graph/sot/torque_control/identification/__init__.py
              python/dynamic_graph/sot/torque_control/identification/identification_utils.py
         DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/identification)
         
INSTALL(FILES python/dynamic_graph/sot/torque_control/utils/__init__.py
              python/dynamic_graph/sot/torque_control/utils/plot_utils.py
              python/dynamic_graph/sot/torque_control/utils/sot_utils.py
         DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/utils)
         
INSTALL(FILES python/dynamic_graph/sot/torque_control/tests/__init__.py
         DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/tests)
         
INSTALL(FILES hrp2-torque-control/python/dynamic_graph/sot/torque_control/hrp2/__init__.py
              hrp2-torque-control/python/dynamic_graph/sot/torque_control/hrp2/motors_parameters.py
              hrp2-torque-control/python/dynamic_graph/sot/torque_control/hrp2/joint_pos_ctrl_gains.py
              hrp2-torque-control/python/dynamic_graph/sot/torque_control/hrp2/balance_ctrl_conf.py
              hrp2-torque-control/python/dynamic_graph/sot/torque_control/hrp2/balance_ctrl_sim_conf.py
         DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/hrp2)

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
