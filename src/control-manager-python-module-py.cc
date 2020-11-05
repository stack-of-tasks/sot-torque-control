#include <dynamic-graph/python/module.hh>
#include "sot/torque_control/control-manager.hh"

namespace dg = dynamicgraph;


BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::sot::torque_control::ControlManager, bp::bases<dg::Entity>, dg::python::AddCommands>() ;
}
