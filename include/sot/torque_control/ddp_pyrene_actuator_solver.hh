/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Noelie RAMUZAT
 * This file is part of sot-torque-control.
 * See license file.
 */

#ifndef _SOT_DDP_PYRENE_ACTUATOR_SOLVER_H
#define _SOT_DDP_PYRENE_ACTUATOR_SOLVER_H
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(sot_ddp_pyrene_actuator_EXPORTS)
#define SOTDDPPYRENEACTUATORSOLVER_EXPORT __declspec(dllexport)
#else
#define SOTDDPPYRENEACTUATORSOLVER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTDDPPYRENEACTUATORSOLVER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <ddp-actuator-solver/ddpsolver.hh>
#include <ddp-actuator-solver/pyrene_actuator/pyreneActuator.hh>
#include <ddp-actuator-solver/pyrene_actuator/pyreneCostFunction.hh>
#include <sot/core/causal-filter.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <tsid/utils/stop-watch.hpp>
#include <vector>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

class SOTDDPPYRENEACTUATORSOLVER_EXPORT DdpPyreneActuatorSolver
    : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Constructor */
  DdpPyreneActuatorSolver(const std::string& name);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(pos_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(pos_joint_measure, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(dx_joint_measure, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(tau_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(tau, dynamicgraph::Vector);

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  dynamicgraph::Vector m_previous_tau;
  double m_dt;
  bool m_initSucceeded;
  DDPSolver<double, 2, 1>::stateVec_t m_zeroState;
  DDPSolver<double, 2, 1>::commandVec_t m_u;
  unsigned int m_T;
  double m_stopCrit;
  unsigned int m_iterMax;
  pyreneActuator m_model;
  CostFunctionPyreneActuator m_cost;
  DDPSolver<double, 2, 1> m_solver;

  /** Initialize the DDP.
   * @param timestep Control period (in seconds).
   * @param T  Size of the preview window (in nb of timestep).
   * @param nbItMax Maximum number of iterations.
   * @param stopCriteria The value of the stopping criteria.
   */
  void param_init(const double& timestep, const int& T, const int& nbItMax,
                  const double& stopCriteria);

  // /* --- SETTER LIM --- */
  void setTorqueLimit(const double& tau);
  void setJointLimit(const double& upperLim, const double& lowerLim);
  void setJointVelLimit(const double& upperLim, const double& lowerLim);

  // /* --- SETTER LOAD --- */
  void setLoadParam(const double& mass, const double& coordX,
                    const double& coordY);
  void setLoadMass(const double& mass);
  void removeLoad();

  // /* --- SETTER GAINS --- */
  void setCostGainState(const dynamicgraph::Vector& Q);
  void setCostGainStateConstraint(const dynamicgraph::Vector& W);
  void setCostGainCommand(const dynamicgraph::Vector& R);
  void setCostGainTorqueConstraint(const dynamicgraph::Vector& P);
};
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
#endif  // _SOT_SIMPLE_DDP_ACTUATOR_SOLVER_H
