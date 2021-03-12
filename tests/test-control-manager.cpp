
/* Copyright 2019, LAAS-CNRS
 *
 * Olivier Stasse
 *
 */
#include <sstream>
#include <iostream>

#include <example-robot-data/path.hpp>

#include <dynamic-graph/factory.h>

// Needed for sendMsg.
#define ENABLE_RT_LOG
#include <dynamic-graph/real-time-logger.h>

#include <sot/torque_control/control-manager.hh>

#include <sot/core/robot-utils.hh>
#include <sot/core/debug.hh>

#define BOOST_TEST_MODULE test - control - manager

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

using boost::test_tools::output_test_stream;
namespace dyn_sot_tc = dynamicgraph::sot::torque_control;

BOOST_AUTO_TEST_CASE(testControlManager) {
  dgADD_OSTREAM_TO_RTLOG(std::cout);

  dyn_sot_tc::ControlManager &a_control_manager = *(dynamic_cast<dyn_sot_tc::ControlManager *>(
      dynamicgraph::FactoryStorage::getInstance()->newEntity("ControlManager", "a_control_manager")));

  a_control_manager.setLoggerVerbosityLevel(dynamicgraph::VERBOSITY_ALL);

  std::string robotRef("simple_humanoid_description");
  a_control_manager.init(0.001, EXAMPLE_ROBOT_DATA_MODEL_DIR "/simple_humanoid_description/urdf/simple_humanoid.urdf", robotRef);

  dynamicgraph::Vector av;
  a_control_manager.m_uSOUT.needUpdate(6);

  dynamicgraph::RealTimeLogger::destroy();
}
