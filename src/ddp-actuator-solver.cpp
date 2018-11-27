/*
 * Copyright 2018-, Olivier Stasse LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/ddp-actuator-solver.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace Eigen;
      
      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef DdpActuatorSolver EntityClassName;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DdpActuatorSolver,"DdpActuatorSolver");

      DdpActuatorSolver::
      DdpActuatorSolver(const std::string &name)
	: Entity(name),
	  CONSTRUCT_SIGNAL_IN (pos_des,           dynamicgraph::Vector),
	  CONSTRUCT_SIGNAL_IN (pos_motor_measure, dynamicgraph::Vector),
	  CONSTRUCT_SIGNAL_IN (pos_joint_measure, dynamicgraph::Vector),
	  CONSTRUCT_SIGNAL_IN (dx_measure,        dynamicgraph::Vector),
	  CONSTRUCT_SIGNAL_IN (tau_measure,       dynamicgraph::Vector),
	  CONSTRUCT_SIGNAL_IN (temp_measure,      dynamicgraph::Vector),
	  CONSTRUCT_SIGNAL_OUT(tau,               dynamicgraph::Vector, m_pos_desSIN),
	  m_T(3000),
	  m_dt(1e-3),
	  m_iterMax(100),
	  m_stopCrit(1e-5),
	  m_solver(m_model,m_cost,
		   DISABLE_FULLDDP,
		   DISABLE_QPBOX)
      {
	Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS );

      m_zeroState.setZero();

      /* Commands. */
	addCommand("init",
		   makeCommandVoid4(*this, &DdpActuatorSolver::param_init,
				    docCommandVoid4("Initialize the DDP solver.",
						    "Control timestep [s].",
						    "Size of the preview window (in nb of samples)",
						    "Max. nb. of iterations",
						    "Stopping criteria")));

      }

      /* --- COMMANDS ---------------------------------------------------------- */
      DEFINE_SIGNAL_OUT_FUNCTION(tau, dynamicgraph::Vector)
      {
	/// ---- Get the information -----
	/// Desired position
	const dynamicgraph::Vector &
	  pos_des = m_pos_desSIN(iter);
	/// Measured position 
	const dynamicgraph::Vector &
	  pos_joint_measure = m_pos_joint_measureSIN(iter);
	/// Measured speed
	const dynamicgraph::Vector &
	  dx_measure = m_dx_measureSIN(iter);
	/// Measured temperature
	const dynamicgraph::Vector &
	  temp_measure = m_temp_measureSIN(iter);
	/// Measured torque
	const dynamicgraph::Vector &
	  tau_measure = m_tau_measureSIN(iter);
	
	DDPSolver<double,5,1>::stateVec_t xinit,xDes;

	/// --- Initialize solver ---
	xinit << pos_joint_measure(0),
	  dx_measure(0),
	  temp_measure(0),
	  tau_measure(0),
	  m_ambiant_temperature;

	xDes << m_pos_desSIN(0), 0.0, 0.0, 0.0, 0.0;

	m_solver.initSolver(xinit,xDes);

	/// --- Solve the DDP --- 
	s = m_solver.solveTrajectory();

	return s;
      }

      void DdpActuatorSolver::
      param_init(const double &timestep,
		 const int &T,
		 const int &nbItMax,
		 const double &stopCriteria)
      {
	m_T = T;
	m_dt = timestep;
	m_iterMax = nbItMax;
	m_stopCrit = stopCriteria;
      m_solver.FirstInitSolver( m_zeroState, m_zeroState,
                                m_T ,m_dt,m_iterMax,m_stopCrit);
      }

      void DdpActuatorSolver::
      display(std::ostream &os) const
      {
	os << " T: " << m_T
	   << " timestep: " << m_dt
	   << " nbItMax: " << m_iterMax
	   << " stopCriteria: "<< m_stopCrit << std::endl;
      }
    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
      
