INSTALL(FILES
  __init__.py
  main.py
  create_entities_utils.py
  main_sim.py
  DESTINATION ${PYTHON_INSTALL_DIR})

INSTALL(FILES
  identification/__init__.py
  identification/identification_utils.py
  DESTINATION ${PYTHON_INSTALL_DIR}/identification)

INSTALL(FILES
  utils/__init__.py
  utils/plot_utils.py
  utils/sot_utils.py
  utils/filter_utils.py
  DESTINATION ${PYTHON_INSTALL_DIR}/utils)

INSTALL(FILES
  tests/__init__.py
  tests/test_control_manager.py
  tests/test_balance_ctrl_openhrp.py
  tests/test_velocity_filters.py
  DESTINATION ${PYTHON_INSTALL_DIR}/tests)


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
    tests/${py_filename}.py.in
    tests/${py_filename}.py
    )

  INSTALL(FILES tests/${py_filename}.py DESTINATION ${PYTHON_INSTALL_DIR}/tests)
ENDFOREACH(py_filename ${${PROJECT_NAME}_PYTHON_CONFIGURABLE_FILES})
