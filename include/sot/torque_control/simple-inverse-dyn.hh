/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_simple_inverse_dyn_H__
#define __sot_torque_control_simple_inverse_dyn_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(simple_inverse_dyn_EXPORTS)
#define SOTSIMPLEINVERSEDYN_EXPORT __declspec(dllexport)
#else
#define SOTSIMPLEINVERSEDYN_EXPORT __declspec(dllimport)
#endif
#else
#define SOTSIMPLEINVERSEDYN_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/spatial/fwd.hpp>
#include <sot/core/robot-utils.hh>
#include <tsid/robots/robot-wrapper.hpp>

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <map>
#include "boost/assign.hpp"

#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>

/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>

#include <sot/torque_control/utils/vector-conversions.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

enum ControlOutput { CONTROL_OUTPUT_VELOCITY = 0, CONTROL_OUTPUT_TORQUE = 1, CONTROL_OUTPUT_SIZE = 2 };

const std::string ControlOutput_s[] = {"velocity", "torque"};

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSIMPLEINVERSEDYN_EXPORT SimpleInverseDyn : public ::dynamicgraph::Entity {
  typedef SimpleInverseDyn EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleInverseDyn(const std::string& name);

  /* --- SIGNALS --- */

  DECLARE_SIGNAL_IN(posture_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_posture, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_posture, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(w_posture, double);

  DECLARE_SIGNAL_IN(kp_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_pos, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(tau_measured, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_tau, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(com_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_com, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_com, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(w_com, double);

  DECLARE_SIGNAL_IN(kp_contact, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_contact, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(w_forces, double);
  DECLARE_SIGNAL_IN(mu, double);
  DECLARE_SIGNAL_IN(contact_points, dynamicgraph::Matrix);
  DECLARE_SIGNAL_IN(contact_normal, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(f_min, double);
  DECLARE_SIGNAL_IN(f_max, double);

  DECLARE_SIGNAL_IN(waist_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(waist_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(waist_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_waist, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_waist, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(w_waist, double);

  DECLARE_SIGNAL_IN(q, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(v, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_measured, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(active_joints, dynamicgraph::Vector);  /// mask with 1 for controlled joints, 0 otherwise
  DECLARE_SIGNAL_INNER(active_joints_checked, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(tau_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dv_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(v_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(q_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(u, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(com, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_pos, dynamicgraph::Vector);

  /* --- COMMANDS --- */

  void init(const double& dt, const std::string& robotRef);
  virtual void setControlOutputType(const std::string& type);
  void updateComOffset(const dynamicgraph::Vector& com_measured);

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  double m_dt;           /// control loop time period
  double m_t;            /// current time
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  bool m_enabled;        /// True if controler is enabled
  bool m_firstTime;      /// True at the first iteration of the controller

  int m_frame_id_rf;  /// frame id of right foot
  int m_frame_id_lf;  /// frame id of left foot
  
  /// TSID
  /// Robot
  tsid::robots::RobotWrapper* m_robot;

  /// Solver and problem formulation
  tsid::solvers::SolverHQPBase* m_hqpSolver;
  tsid::InverseDynamicsFormulationAccForce* m_invDyn;

  /// TASKS
  tsid::contacts::Contact6d* m_contactRF;
  tsid::contacts::Contact6d* m_contactLF;
  tsid::tasks::TaskComEquality* m_taskCom;
  tsid::tasks::TaskSE3Equality* m_taskWaist;
  tsid::tasks::TaskJointPosture* m_taskPosture;
  tsid::tasks::TaskJointPosture* m_taskBlockedJoints;

  /// Trajectories of the tasks
  tsid::trajectories::TrajectorySample m_sampleCom;
  tsid::trajectories::TrajectorySample m_sampleWaist;
  tsid::trajectories::TrajectorySample m_samplePosture;

  /// Weights of the Tasks (which can be changed)
  double m_w_com;
  double m_w_posture;
  double m_w_waist;

  /// Computed solutions (accelerations and torques) and their derivatives
  tsid::math::Vector m_dv_sot;   /// desired accelerations (sot order)
  tsid::math::Vector m_dv_urdf;  /// desired accelerations (urdf order)
  tsid::math::Vector m_v_sot;    /// desired velocities (sot order)
  tsid::math::Vector m_v_urdf;   /// desired and current velocities (urdf order) (close the TSID loop on it)
  tsid::math::Vector m_q_sot;    /// desired positions (sot order)
  tsid::math::Vector m_q_urdf;   /// desired and current positions (urdf order) (close the TSID loop on it)
  tsid::math::Vector m_tau_sot;  /// desired torques (sot order)

  tsid::math::Vector3 m_com_offset;  /// 3d CoM offset

  unsigned int m_timeLast;       /// Final time of the control loop
  RobotUtilShrPtr m_robot_util;  /// Share pointer to the robot utils methods
  ControlOutput m_ctrlMode;      /// ctrl mode desired for the output (velocity or torque)

};  // class SimpleInverseDyn
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_simple_inverse_dyn_H__
