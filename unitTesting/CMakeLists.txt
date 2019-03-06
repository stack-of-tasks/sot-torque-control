# Add unit tests for validation

IF(BUILD_PYTHON_INTERFACE)
  SET(LIST_OF_TESTS
    unit_test_control_manager.py
    unit_test_free_flyer_locator.py
    unit_test_position_controller.py
    unit_test_se3_trajectory_generator.py
    )

  foreach(localtest ${LIST_OF_TESTS})
#    ADD_PYTHON_UNIT_TEST(${localtest} unitTesting/${localtest} "src")
  endforeach()
ENDIF(BUILD_PYTHON_INTERFACE)
