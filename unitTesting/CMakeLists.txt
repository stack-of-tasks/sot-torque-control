# Add unit tests for validation

SET(LIST_OF_TESTS
  unit_test_control_manager.py
  unit_test_inverse_dynamics_balance_controller.py
  unit_test_free_flyer_locator.py
  unit_test_position_controller.py
)

foreach(localtest ${LIST_OF_TESTS})
  ADD_TEST(${localtest} ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${localtest})
endforeach()



