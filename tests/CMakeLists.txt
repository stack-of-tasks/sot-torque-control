# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

SET(urdfpath ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/urdf/simple_humanoid.urdf)

# Add unit tests for validation
MACRO(SOT_TOR_CTR_TEST NAME)
  ADD_UNIT_TEST(test-${NAME} test-${NAME}.cpp)

  TARGET_LINK_LIBRARIES(test-${NAME} PRIVATE ${PROJECT_NAME} Boost::unit_test_framework ${NAME}
    dynamic-graph::dynamic-graph pinocchio::pinocchio)
  TARGET_COMPILE_DEFINITIONS(test-${NAME} PUBLIC URDF_FULL_PATH="${urdfpath}")
ENDMACRO(SOT_TOR_CTR_TEST)

SOT_TOR_CTR_TEST(control-manager)

IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON_TESTS
    unit_test_control_manager.py
    unit_test_free_flyer_locator.py
    unit_test_position_controller.py
    unit_test_se3_trajectory_generator.py
    )

  foreach(localtest ${${PROJECT_NAME}_PYTHON_TESTS})
    ADD_PYTHON_UNIT_TEST(${localtest} tests/${localtest})
  endforeach()
ENDIF(BUILD_PYTHON_INTERFACE)
