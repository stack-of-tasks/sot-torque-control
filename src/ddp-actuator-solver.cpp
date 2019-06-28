/*
 * Copyright 2018-, Olivier Stasse LAAS-CNRS
 *
 */

#include <Eigen/Dense>
#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/ddp-actuator-solver.hh>

#include <ddp-actuator-solver/examples/dctemp.hh>
#include <ddp-actuator-solver/examples/costtemp.hh>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/debug-ddp_actuator_solver.dat"

#define RESETDEBUG5()                            \
  {                                              \
    std::ofstream DebugFile;                     \
    DebugFile.open(DBGFILE, std::ofstream::out); \
    DebugFile.close();                           \
  }
#define ODEBUG5FULL(x)                                                                          \
  {                                                                                             \
    std::ofstream DebugFile;                                                                    \
    DebugFile.open(DBGFILE, std::ofstream::app);                                                \
    DebugFile << __FILE__ << ":" << __FUNCTION__ << "(#" << __LINE__ << "):" << x << std::endl; \
    DebugFile.close();                                                                          \
  }
#define ODEBUG5(x)                               \
  {                                              \
    std::ofstream DebugFile;                     \
    DebugFile.open(DBGFILE, std::ofstream::app); \
    DebugFile << x << std::endl;                 \
    DebugFile.close();                           \
  }

#define RESETDEBUG4()
#define ODEBUG4FULL(x)
#define ODEBUG4(x)

namespace dynamicgraph {
namespace sot {
namespace torque_control {

namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace Eigen;

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef DdpActuatorSolver EntityClassName;

/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DdpActuatorSolver, "DdpActuatorSolver");

DdpActuatorSolver::DdpActuatorSolver(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(pos_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(pos_motor_measure, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(pos_joint_measure, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dx_measure, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau_measure, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(tau_des, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(temp_measure, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(tau, dynamicgraph::Vector, m_pos_desSIN),
      m_dt(1e-3),
      m_solver(m_model, m_cost, DISABLE_FULLDDP, DISABLE_QPBOX),
      m_T(3000),
      m_stopCrit(1e-5),
      m_iterMax(100) {
  RESETDEBUG5();
  Entity::signalRegistration(ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);

  m_zeroState.setZero();

  /* Commands. */
  addCommand("init", makeCommandVoid4(*this, &DdpActuatorSolver::param_init,
                                      docCommandVoid4("Initialize the DDP solver.", "Control timestep [s].",
                                                      "Size of the preview window (in nb of samples)",
                                                      "Max. nb. of iterations", "Stopping criteria")));
}

/* --- COMMANDS ---------------------------------------------------------- */
DEFINE_SIGNAL_OUT_FUNCTION(tau, dynamicgraph::Vector) {
  /// ---- Get the information -----
  /// Desired position
  const dynamicgraph::Vector &pos_des = m_pos_desSIN(iter);
  /// Measured position
  const dynamicgraph::Vector &pos_joint_measure = m_pos_joint_measureSIN(iter);
  /// Measured speed
  const dynamicgraph::Vector &dx_measure = m_dx_measureSIN(iter);
  /// Measured temperature
  const dynamicgraph::Vector &temp_measure = m_temp_measureSIN(iter);
  /// Measured torque
  const dynamicgraph::Vector &tau_measure = m_tau_measureSIN(iter);
  /// Desired torque
  const dynamicgraph::Vector &tau_des = m_tau_desSIN(iter);

  DDPSolver<double, 5, 1>::stateVec_t xinit, xDes;

  /// --- Initialize solver ---
  xinit << pos_joint_measure(0), dx_measure(0), temp_measure(0), tau_measure(0),
      // m_ambiant_temperature;
      25.0;

  xDes << pos_des, 0.0, 25.0, tau_des, 25.0;
  ODEBUG5(xinit);
  ODEBUG5("");
  ODEBUG5(xDes);

  DCTemp model;
  CostTemp cost;
  DDPSolver<double, 5, 1> m_solver(model, cost, 0, 0);

  m_solver.FirstInitSolver(xinit, xDes, m_T, m_dt, m_iterMax, m_stopCrit);
  ODEBUG5("FirstInitSolver");

  /// --- Solve the DDP ---
  m_solver.solveTrajectory();
  ODEBUG5("Trajectory solved");

  /// --- Get the command ---
  DDPSolver<double, 5, 1>::traj lastTraj;
  lastTraj = m_solver.getLastSolvedTrajectory();
  ODEBUG5("getLastSolvedTrajectory");

  DDPSolver<double, 5, 1>::commandVecTab_t uList;
  uList = lastTraj.uList;
  ODEBUG5("uList");

  // s = uList[0];
  s.resize(32);
  s << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, uList[0], 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // s.setZero(32);
  ODEBUG5(s);
  return s;
}

void DdpActuatorSolver::param_init(const double &timestep, const int &T, const int &nbItMax,
                                   const double &stopCriteria) {
  m_T = T;
  m_dt = timestep;
  m_iterMax = nbItMax;
  m_stopCrit = stopCriteria;
  m_solver.FirstInitSolver(m_zeroState, m_zeroState, m_T, m_dt, m_iterMax, m_stopCrit);
}

void DdpActuatorSolver::display(std::ostream &os) const {
  os << " T: " << m_T << " timestep: " << m_dt << " nbItMax: " << m_iterMax << " stopCriteria: " << m_stopCrit
     << std::endl;
}
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
