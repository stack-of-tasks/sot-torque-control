
/* Copyright 2019, LAAS-CNRS
 *
 * Olivier Stasse
 *
 */
#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <sot/torque_control/control-manager.hh>
#include <sstream>

using boost::test_tools::output_test_stream;
namespace dyn_sot_tc = dynamicgraph::sot::torque_control;

BOOST_AUTO_TEST_CASE(testControlManager) {
  dyn_sot_tc::ControlManager &a_control_manager = *(dynamic_cast<dyn_sot_tc *>(
      dynamicgraph::FactoryStorage::getInstance()->newEntity(
          "ControlManager", "a_control_manager")));
}
