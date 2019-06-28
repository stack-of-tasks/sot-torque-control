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

#include <Eigen/Dense>
#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/ddp_pyrene_actuator_solver.hh>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/debug-ddp_pyrene_actuator_solver.dat"

#define RESETDEBUG5() { std::ofstream DebugFile;  \
    DebugFile.open(DBGFILE,std::ofstream::out);   \
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
    DebugFile << __FILE__ << ":"      \
              << __FUNCTION__ << "(#"     \
              << __LINE__ << "):" << x << std::endl;  \
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;    \
    DebugFile.close();}

#define RESETDEBUG4()
#define ODEBUG4FULL(x)
#define ODEBUG4(x)

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

#define ALL_INPUT_SIGNALS m_pos_desSIN << m_pos_joint_measureSIN << m_dx_joint_measureSIN 

#define ALL_OUTPUT_SIGNALS  m_tauSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef DdpPyreneActuatorSolver EntityClassName;

/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DdpPyreneActuatorSolver, "DdpPyreneActuatorSolver");

DdpPyreneActuatorSolver::
DdpPyreneActuatorSolver(const std::string &name)
  : Entity(name),
    CONSTRUCT_SIGNAL_IN (pos_des,           dynamicgraph::Vector),
    CONSTRUCT_SIGNAL_IN (pos_joint_measure, dynamicgraph::Vector),
    CONSTRUCT_SIGNAL_IN (dx_joint_measure,  dynamicgraph::Vector),
    CONSTRUCT_SIGNAL_OUT(tau,               dynamicgraph::Vector, m_pos_desSIN),
    m_dt(1e-3),
    m_T(50),
    m_stopCrit(1e-3),
    m_iterMax(100),
    m_solver(m_model, m_cost,
             DISABLE_FULLDDP,
             DISABLE_QPBOX)
    {

      RESETDEBUG5();
      Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS );

      m_zeroState.setZero();

      /* Commands. */
      addCommand("init",
                 makeCommandVoid4(*this, &DdpPyreneActuatorSolver::param_init,
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
  /// Measured joint position
  const dynamicgraph::Vector &
  pos_joint_measure = m_pos_joint_measureSIN(iter);
  /// Measured joint speed
  const dynamicgraph::Vector &
  dx_joint_measure = m_dx_joint_measureSIN(iter);

  m_xinit << pos_joint_measure(0),
             dx_joint_measure(0);

  m_xDes << pos_des, 0.0;
  

  m_solver.FirstInitSolver(m_xinit, m_xDes, m_T, m_dt, m_iterMax, m_stopCrit);
  ODEBUG5("FirstInitSolver");

  /// --- Solve the DDP ---
  m_solver.solveTrajectory();
  ODEBUG5("Trajectory solved");

  /// --- Get the command ---
  DDPSolver<double, 2, 1>::traj lastTraj;
  lastTraj = m_solver.getLastSolvedTrajectory();
  ODEBUG5("getLastSolvedTrajectory");

  DDPSolver<double, 2, 1>::commandVecTab_t uList;
  uList = lastTraj.uList;
  ODEBUG5("uList");

  //s = uList[0];
  s.resize(32);
  s << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, uList[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  //s.setZero(32);
  ODEBUG5(s);
  return s;
}

void DdpPyreneActuatorSolver::param_init(const double &timestep,
                                   const int &T,
                                   const int &nbItMax,
                                   const double &stopCriteria) 
{
  m_T = T;
  m_dt = timestep;
  m_iterMax = nbItMax;
  m_stopCrit = stopCriteria;
  m_cost.setTauLimit(70);
  m_cost.setJointLimit(0.0, -2.35619449019);
  m_cost.setJointVelLimit(30.0, -30.0);
  m_solver.FirstInitSolver( m_zeroState, m_zeroState,
                            m_T , m_dt, m_iterMax, m_stopCrit);
}

void DdpPyreneActuatorSolver::display(std::ostream &os) const 
{
  os << " T: " << m_T
     << " timestep: " << m_dt
     << " nbItMax: " << m_iterMax
     << " stopCriteria: " << m_stopCrit << std::endl;
}

} // namespace torque_control
} // namespace sot
} // namespace dynamicgraph
