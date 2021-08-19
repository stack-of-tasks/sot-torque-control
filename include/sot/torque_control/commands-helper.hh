/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 */

#ifndef __sot_torquecontrol_commands_helper_H__
#define __sot_torquecontrol_commands_helper_H__

#include <boost/function.hpp>

/* --- COMMON INCLUDE -------------------------------------------------- */
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>

/* --- HELPER ---------------------------------------------------------- */
namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;
namespace torquecontrol {
using ::dynamicgraph::command::docCommandVerbose;
using ::dynamicgraph::command::docCommandVoid0;
using ::dynamicgraph::command::docCommandVoid1;
using ::dynamicgraph::command::docCommandVoid2;
using ::dynamicgraph::command::docCommandVoid3;
using ::dynamicgraph::command::docCommandVoid4;
using ::dynamicgraph::command::docCommandVoid5;
using ::dynamicgraph::command::docCommandVoid6;
using ::dynamicgraph::command::docCommandVoid7;
using ::dynamicgraph::command::docCommandVoid8;
using ::dynamicgraph::command::docDirectGetter;
using ::dynamicgraph::command::docDirectSetter;
using ::dynamicgraph::command::makeCommandVerbose;
using ::dynamicgraph::command::makeCommandVoid0;
using ::dynamicgraph::command::makeCommandVoid1;
using ::dynamicgraph::command::makeCommandVoid2;
using ::dynamicgraph::command::makeCommandVoid3;
using ::dynamicgraph::command::makeCommandVoid4;
using ::dynamicgraph::command::makeCommandVoid5;
using ::dynamicgraph::command::makeCommandVoid6;
using ::dynamicgraph::command::makeCommandVoid7;
using ::dynamicgraph::command::makeCommandVoid8;
using ::dynamicgraph::command::makeDirectGetter;
using ::dynamicgraph::command::makeDirectSetter;
}  // namespace torquecontrol
}  // namespace sot
}  // namespace dynamicgraph

#endif  // __sot_torquecontrol_commands_helper_H__
