# Copyright 2014, 2020, Andrea Del Prete, Guilhem Saurel, LAAS-CNRS
#

set(plugins
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
    trace-player)

if(DDP_ACTUATOR_SOLVER_FOUND)
  set(plugins ${plugins} ddp-actuator-solver)
  set(plugins ${plugins} ddp_pyrene_actuator_solver)
endif(DDP_ACTUATOR_SOLVER_FOUND)

foreach(plugin ${plugins})
  get_filename_component(LIBRARY_NAME ${plugin} NAME)
  add_library(${LIBRARY_NAME} SHARED "${plugin}.cpp")

  if(SUFFIX_SO_VERSION)
    set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  target_link_libraries(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME}
                                               ${${LIBRARY_NAME}_deps})

  if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    install(
      TARGETS ${LIBRARY_NAME}
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  if(BUILD_PYTHON_INTERFACE)
    string(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      dynamic_graph_python_module(
        "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      dynamic_graph_python_module(
        "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  endif(BUILD_PYTHON_INTERFACE)
endforeach(plugin)
