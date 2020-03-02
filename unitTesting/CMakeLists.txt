# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Add unit tests for validation

MACRO(SOT_TOR_CTR_TEST NAME)
  ADD_EXECUTABLE(test-${NAME} test-${NAME}.cpp)
  IF (test-${NAME}-args)
    ADD_TEST(test-${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/test-${NAME} ${test-${NAME}-args} )
  ELSE (test-${NAME}-args)
    ADD_TEST(test-${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/test-${NAME} )
  ENDIF (test-${NAME}-args)

  TARGET_LINK_LIBRARIES(test-${NAME} ${PROJECT_NAME}
    dynamic-graph::dynamic-graph
    pinocchio::pinocchio
    ${Boost_LIBRARIES})
  ADD_DEPENDENCIES(test-${NAME} ${PROJECT_NAME})

  IF (test-${NAME}-libs)
    TARGET_LINK_LIBRARIES(test-${NAME} ${test-${NAME}-libs})
    ADD_DEPENDENCIES(test-${NAME} ${test-${NAME}-libs})
  ENDIF (test-${NAME}-libs)
ENDMACRO(SOT_TOR_CTR_TEST)

SET(urdfpath
  ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/urdf/simple_humanoid.urdf
  )
SET(srdfpath
  ${SIMPLE_HUMANOID_DESCRIPTION_PKGDATAROOTDIR}/simple_humanoid_description/srdf/simple_humanoid.srdf
  )

SET(test-control-manager-args ${urdfpath} ${srdfpath} )
SET(test-control-manager-libs control-manager)
SOT_TOR_CTR_TEST(control-manager)

IF(BUILD_PYTHON_INTERFACE)
  SET(LIST_OF_TESTS
    unit_test_control_manager.py
    unit_test_free_flyer_locator.py
    unit_test_position_controller.py
    unit_test_se3_trajectory_generator.py
    )

  foreach(localtest ${LIST_OF_TESTS})
    ADD_PYTHON_UNIT_TEST(${localtest} unitTesting/${localtest})
  endforeach()
ENDIF(BUILD_PYTHON_INTERFACE)
