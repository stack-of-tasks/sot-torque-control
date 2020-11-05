# Copyright 2014, 2020, Andrea Del Prete, Guilhem Saurel, LAAS-CNRS
#

SET(plugins
  admittance-controller
  base-estimator
  control-manager
  current-controller
  device-torque-ctrl
  free-flyer-locator
  imu_offset_compensation
  inverse-dynamics-balance-controller
  simple-inverse-dyn
  joint-torque-controller
  joint-trajectory-generator
  numerical-difference
  position-controller
  se3-trajectory-generator
  torque-offset-estimator
  trace-player
  ddp-actuator-solver
  ddp_pyrene_actuator_solver
  )

FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin})

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME} ${${LIBRARY_NAME}_deps})

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        SOURCE_PYTHON_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        MODULE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

IF(BUILD_PYTHON_INTERFACE)
  #SET(NO_INSTALL_OF_INIT_PY 0)
  #DYNAMIC_GRAPH_PYTHON_MODULE(${PYTHON_DIR} ${PROJECT_NAME} wrap ${NO_INSTALL_OF_INIT_PY})

  # Install empty __init__.py files in intermediate directories.
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/${PYTHON_DIR}/__init__.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/${PYTHON_DIR}
    )
ENDIF(BUILD_PYTHON_INTERFACE)
