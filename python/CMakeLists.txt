set(${PROJECT_NAME}_PYTHON_FILES __init__.py main.py create_entities_utils.py
                                 main_sim.py)

set(${PROJECT_NAME}_PYTHON_IDENTIFICATION __init__.py identification_utils.py)

set(${PROJECT_NAME}_PYTHON_UTILS __init__.py plot_utils.py sot_utils.py
                                 filter_utils.py)

set(${PROJECT_NAME}_PYTHON_TESTS
    __init__.py robot_data_test.py test_balance_ctrl_openhrp.py
    test_control_manager.py test_torque_offset_estimator.py
    test_velocity_filters.py)

foreach(file ${${PROJECT_NAME}_PYTHON_FILES})
  python_install_on_site("dynamic_graph/${PYTHON_DIR}" ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON_FILES})

foreach(file ${${PROJECT_NAME}_PYTHON_IDENTIFICATION})
  python_install_on_site("dynamic_graph/${PYTHON_DIR}/identification" ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON_IDENTIFICATION})

foreach(file ${${PROJECT_NAME}_PYTHON_UTILS})
  python_install_on_site("dynamic_graph/${PYTHON_DIR}/utils" ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON_UTILS})

foreach(file ${${PROJECT_NAME}_PYTHON_TESTS})
  python_install_on_site("dynamic_graph/${PYTHON_DIR}/tests" ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON_TESTS})
