/*
 * Copyright 2010-2017,
 * Florent Lamiraux, Rohan Budhiraja
 *
 * CNRS/AIST
 *
 */

#ifndef STC_COMMAND_HH
#define STC_COMMAND_HH

#include <fstream>
#include <boost/assign/list_of.hpp>

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <sot/torque_control/joint-trajectory-generator.hh>

namespace dynamicgraph { namespace sot {
    namespace command {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;
      using ::dynamicgraph::sot::torque_control::JointTrajectoryGenerator;
      // Command DisplayModel
      class IsTrajectoryEnded : public Command
      {
      public:
        virtual ~IsTrajectoryEnded() {	sotDEBUGIN(15);
          sotDEBUGOUT(15);}
        /// Create command and store it in Entity
        /// \param entity instance of Entity owning this command
        /// \param docstring documentation of the command
        IsTrajectoryEnded(JointTrajectoryGenerator& entity, const std::string& docstring) :
          Command(entity, std::vector<Value::Type>(),
                  docstring)
        {
        }
        virtual Value doExecute()
        {
          JointTrajectoryGenerator& jtg = static_cast<JointTrajectoryGenerator&>(owner());
          bool output = jtg.isTrajectoryEnded();
          return Value(output);
        }
      }; // class IsTrajectoryEnded

    } // namespace command
  } /* namespace sot */
} /* namespace dynamicgraph */

#endif //STC_COMMAND_HH
