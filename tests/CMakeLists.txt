# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Add unit tests for validation
macro(SOT_TOR_CTR_TEST NAME)
  add_unit_test(test-${NAME} test-${NAME}.cpp)

  target_link_libraries(
    test-${NAME}
    PRIVATE ${PROJECT_NAME} Boost::unit_test_framework ${NAME}
            dynamic-graph::dynamic-graph pinocchio::pinocchio
            example-robot-data::example-robot-data)
endmacro(SOT_TOR_CTR_TEST)

sot_tor_ctr_test(control-manager)

if(BUILD_PYTHON_INTERFACE)
  set(${PROJECT_NAME}_PYTHON_TESTS
      unit_test_control_manager.py unit_test_free_flyer_locator.py
      unit_test_position_controller.py unit_test_se3_trajectory_generator.py)

  foreach(localtest ${${PROJECT_NAME}_PYTHON_TESTS})
    add_python_unit_test(${localtest} tests/${localtest})
  endforeach()
endif(BUILD_PYTHON_INTERFACE)
