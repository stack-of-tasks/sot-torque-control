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
  robot_data_test.py
  test_balance_ctrl_openhrp.py
  test_control_manager.py
  test_torque_offset_estimator.py
  test_velocity_filters.py
  )

FOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})

FOREACH(file ${${PROJECT_NAME}_PYTHON_IDENTIFICATION})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/identification" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_IDENTIFICATION})

FOREACH(file ${${PROJECT_NAME}_PYTHON_UTILS})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/utils" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_UTILS})

FOREACH(file ${${PROJECT_NAME}_PYTHON_TESTS})
  PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/tests" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_TESTS})
