SET(${LIBRARY_NAME}_HEADERS
  joint-torque-controller.hh
  base-estimator.hh
  numerical-difference.hh
  motor-model.hh
  joint-trajectory-generator.hh
  se3-trajectory-generator.hh
  free-flyer-locator.hh
  inverse-dynamics-balance-controller.hh
  position-controller.hh
  control-manager.hh
  current-controller.hh
  commands-helper.hh
  device-torque-ctrl.hh
  trace-player.hh
  torque-offset-estimator.hh
  imu_offset_compensation.hh
  admittance-controller.hh
  utils/trajectory-generators.hh
  utils/lin-estimator.hh
  utils/poly-estimator.hh
  utils/quad-estimator.hh
  utils/Stdafx.hh
  utils/vector-conversions.hh
  )

INSTALL(FILES ${${LIBRARY_NAME}_HEADERS} DESTINATION include/${CUSTOM_HEADER_DIR})
