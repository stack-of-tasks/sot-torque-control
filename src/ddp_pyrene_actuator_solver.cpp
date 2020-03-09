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

#define EIGEN_NO_MALLOC
#include <Eigen/Dense>
#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/ddp_pyrene_actuator_solver.hh>
#include <math.h>

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

#define ALL_INPUT_SIGNALS m_pos_desSIN << m_pos_joint_measureSIN << m_dx_joint_measureSIN << m_tau_desSIN

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
    CONSTRUCT_SIGNAL_IN (tau_des,           dynamicgraph::Vector),
    CONSTRUCT_SIGNAL_OUT(tau,               dynamicgraph::Vector, ALL_INPUT_SIGNALS),
    m_dt(1e-3),
    m_T(50),
    m_stopCrit(1e-3),
    m_iterMax(10),
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
      addCommand("setTorqueLimit",
           makeCommandVoid1(*this, &DdpPyreneActuatorSolver::setTorqueLimit,
                            docCommandVoid1("Set the Torque limit.",
                                "Limit of the motor torque.")));
      addCommand("setJointLimit",
           makeCommandVoid2(*this, &DdpPyreneActuatorSolver::setJointLimit,
                            docCommandVoid2("Set the angular limits of the joint.",
                                "Upper limit",
                                "Lower limit.")));
      addCommand("setJointVelLimit",
           makeCommandVoid2(*this, &DdpPyreneActuatorSolver::setJointVelLimit,
                            docCommandVoid2("Set the angular velocity limits of the joint.",
                                "Upper limit",
                                "Lower limit.")));
      addCommand("setLoadParam",
           makeCommandVoid3(*this, &DdpPyreneActuatorSolver::setLoadParam,
                            docCommandVoid3("Setter of the Load parameters.",
                                "Mass of the load [g].",
                                "X coordinate of the Load",
                                "Y coordinate of the Load")));
      addCommand("setLoadMass",
           makeCommandVoid1(*this, &DdpPyreneActuatorSolver::setLoadMass,
                            docCommandVoid1("Set the Load mass.",
                                "Mass of the load [g].")));
      addCommand("removeLoad",
            makeCommandVoid0(*this, &DdpPyreneActuatorSolver::removeLoad,
                        docCommandVoid0("Remove the Load.")));

      addCommand("setCostGainState",
           makeCommandVoid1(*this, &DdpPyreneActuatorSolver::setCostGainState,
                            docCommandVoid1("Set the Gain of the state cost matrix.",
                                "Matrix of Gains.")));
      addCommand("setCostGainCommand",
           makeCommandVoid1(*this, &DdpPyreneActuatorSolver::setCostGainCommand,
                            docCommandVoid1("Set the Gain of the command cost matrix.",
                                "Matrix of Gains.")));
      addCommand("setCostGainStateConstraint",
           makeCommandVoid1(*this, &DdpPyreneActuatorSolver::setCostGainStateConstraint,
                            docCommandVoid1("Set the Gain of the constraints on the state.",
                                "Matrix of Gains.")));
      addCommand("setCostGainTorqueConstraint",
           makeCommandVoid1(*this, &DdpPyreneActuatorSolver::setCostGainTorqueConstraint,
                            docCommandVoid1("Set the Gain of the torque constraints.",
                                "Matrix of Gains.")));

      m_initSucceeded = true;
    }

/* --- COMMANDS ---------------------------------------------------------- */
DEFINE_SIGNAL_OUT_FUNCTION(tau, dynamicgraph::Vector) 
{
    if (!m_initSucceeded)
    {
      SEND_WARNING_STREAM_MSG("Cannot compute signal tau before initialization!");
      return s;
    }
    
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
  /// Desired torque
  const dynamicgraph::Vector &
  tau_des = m_tau_desSIN(iter);

  DDPSolver<double, 2, 1>::stateVec_t xinit, xDes;

  xinit << pos_joint_measure(0),
           dx_joint_measure(0);

  xDes << pos_des, 0.0;
  //ODEBUG(xDes);

  m_solver.initSolver(xinit, xDes);
  //ODEBUG("FirstInitSolver");

  /// --- Solve the DDP ---
  m_solver.solveTrajectory();
  //ODEBUG("Trajectory solved");

  /// --- Get the command ---
  DDPSolver<double, 2, 1>::traj lastTraj;
  lastTraj = m_solver.getLastSolvedTrajectory();
  //ODEBUG("getLastSolvedTrajectory");

  DDPSolver<double, 2, 1>::commandVecTab_t uList;
  uList = lastTraj.uList;

  s = m_previous_tau;
  double tau_ddp = uList[0](0,0);
  if (!isnan(tau_ddp))
  {    
    s[25] = tau_ddp;
  }
  
  m_previous_tau = s;
  //ODEBUG(s);

  return s;
}

void DdpPyreneActuatorSolver::param_init(const double &timestep,
                                   const int &T,
                                   const int &nbItMax,
                                   const double &stopCriteria) 
{
  if (!m_pos_desSIN.isPlugged())
    return SEND_MSG("Init failed: signal pos_des is not plugged", MSG_TYPE_ERROR);
  if (!m_pos_joint_measureSIN.isPlugged())
    return SEND_MSG("Init failed: signal pos_joint_measure is not plugged", MSG_TYPE_ERROR);
  if (!m_dx_joint_measureSIN.isPlugged())
    return SEND_MSG("Init failed: signal dx_joint_measure is not plugged", MSG_TYPE_ERROR);

  m_previous_tau.resize(32);
  m_previous_tau.setZero();
  m_T = T;
  m_dt = timestep;
  m_iterMax = nbItMax;
  m_stopCrit = stopCriteria;
  m_cost.setTauLimit(70.0);
  m_cost.setJointLimit(0.0, -2.35619449019);
  m_cost.setJointVelLimit(30.0, -30.0);
  m_solver.FirstInitSolver( m_zeroState, m_zeroState,
                            m_T , m_dt, m_iterMax, m_stopCrit);
}

void DdpPyreneActuatorSolver::setTorqueLimit(const double& tau)
{
  m_cost.setTauLimit(tau);
}

void DdpPyreneActuatorSolver::setJointLimit(const double& upperLim, const double& lowerLim)
{
  m_cost.setJointLimit(upperLim, lowerLim);
}

void DdpPyreneActuatorSolver::setJointVelLimit(const double& upperLim, const double& lowerLim)
{
  m_cost.setJointVelLimit(upperLim, lowerLim);
}

void DdpPyreneActuatorSolver::setLoadParam(const double& mass, const double& coordX, const double& coordY)
{
  m_model.setLoadParam(mass, coordX, coordY);
}

void DdpPyreneActuatorSolver::setLoadMass(const double& mass)
{
  m_model.setLoadMass(mass);
}

void DdpPyreneActuatorSolver::removeLoad()
{
  m_model.removeLoad();
}

void DdpPyreneActuatorSolver::setCostGainState(const dynamicgraph::Vector& Q)
{
  const CostFunction<double,2,1>::stateMat_t Q_new = Eigen::Map<const CostFunction<double,2,1>::stateMat_t, Eigen::Unaligned >(Q.data(),2,1);
  m_cost.setCostGainState(Q_new);
}

void DdpPyreneActuatorSolver::setCostGainStateConstraint(const dynamicgraph::Vector& W)
{
  const CostFunction<double,2,1>::stateMat_t W_new = Eigen::Map<const CostFunction<double,2,1>::stateMat_t, Eigen::Unaligned >(W.data(),2,1);
  m_cost.setCostGainStateConstraint(W_new);
}

void DdpPyreneActuatorSolver::setCostGainCommand(const dynamicgraph::Vector& R)
{
  const CostFunction<double,2,1>::commandMat_t R_new = Eigen::Map<const CostFunction<double,2,1>::commandMat_t, Eigen::Unaligned >(R.data(),1);
  m_cost.setCostGainCommand(R_new);
}

void DdpPyreneActuatorSolver::setCostGainTorqueConstraint(const dynamicgraph::Vector& P)
{
  const CostFunction<double,2,1>::commandMat_t P_new = Eigen::Map<const CostFunction<double,2,1>::commandMat_t, Eigen::Unaligned >(P.data(),1);
  m_cost.setCostGainTorqueConstraint(P_new);
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
