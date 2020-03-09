SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  main.py
  create_entities_utils.py
  main_sim.py
  )

SET(${PROJECT_NAME}_PYTHON_IDENTIFICATION
  __init__.py
  identification_utils.py
  )

SET(${PROJECT_NAME}_PYTHON_UTILS
  __init__.py
  plot_utils.py
  sot_utils.py
  filter_utils.py
  )

SET(${PROJECT_NAME}_PYTHON_TESTS
  __init__.py
  test_control_manager.py
  test_balance_ctrl_openhrp.py
  test_velocity_filters.py
  )

FOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})

FOREACH(file ${${PROJECT_NAME}_PYTHON_IDENTIFICATION})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/identification" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_IDENTIFICATION})

FOREACH(file ${${PROJECT_NAME}_PYTHON_UTILS})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/utils"${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_UTILS})

FOREACH(file ${${PROJECT_NAME}_PYTHON_TESTS})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/tests" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_TESTS})


SET(${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES)

IF(TALOS_DATA_FOUND)
  SET(${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES
    ${${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES}
    test_torque_offset_estimator
    )
ENDIF(TALOS_DATA_FOUND)

IF(SIMPLE_HUMANOID_DESCRIPTION_FOUND)
  SET(${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES
    ${${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES}
    robot_data_test
    )
ENDIF(SIMPLE_HUMANOID_DESCRIPTION_FOUND)

FOREACH(py_filename ${${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES})
  CONFIGURE_FILE(
    "dynamic_graph/${PYTHON_DIR}/tests/${py_filename}.py.in"
    "dynamic_graph/${PYTHON_DIR}/tests/${py_filename}.py"
    )

  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/tests" ${py_filename}.py)
ENDFOREACH(py_filename ${${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES})
