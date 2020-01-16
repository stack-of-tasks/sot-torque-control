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

#if defined (WIN32)
#  if defined (simple_inverse_dyn_EXPORTS)
#    define SOTSIMPLEINVERSEDYN_EXPORT __declspec(dllexport)
#  else
#    define SOTSIMPLEINVERSEDYN_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSIMPLEINVERSEDYN_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <map>
#include "boost/assign.hpp"

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>

/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>

#include <sot/torque_control/utils/vector-conversions.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSIMPLEINVERSEDYN_EXPORT SimpleInverseDyn
  : public::dynamicgraph::Entity {
  typedef SimpleInverseDyn EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleInverseDyn( const std::string & name );

  void init(const double& dt,
            const std::string& robotRef);
  // void updateComOffset();

  /* --- SIGNALS --- */

  DECLARE_SIGNAL_IN(posture_ref_pos,            dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_vel,            dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_acc,            dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_posture,                 dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_posture,                 dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(w_posture,                  double);

  DECLARE_SIGNAL_IN(kp_pos,                     dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_pos,                     dynamicgraph::Vector);
  
  DECLARE_SIGNAL_IN(com_ref_pos,                dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_ref_vel,                dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_ref_acc,                dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_com,                     dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_com,                     dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(w_com,                      double);

  DECLARE_SIGNAL_IN(q,                          dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(v,                          dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(active_joints,              dynamicgraph::Vector); /// mask with 1 for controlled joints, 0 otherwise
  DECLARE_SIGNAL_INNER(active_joints_checked,   dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(tau_des,                   dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dv_des,                    dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(u,                         dynamicgraph::Vector);


  // DECLARE_SIGNAL_IN(waist_ref_pos,              dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(waist_ref_vel,              dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(waist_ref_acc,              dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(rf_ref_pos,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(rf_ref_vel,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(rf_ref_acc,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(lf_ref_pos,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(lf_ref_vel,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(lf_ref_acc,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(f_ref_right_foot,           dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(f_ref_left_foot,            dynamicgraph::Vector);

  // DECLARE_SIGNAL_IN(kp_contact,                 dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(kd_contact,                 dynamicgraph::Vector);

  // DECLARE_SIGNAL_IN(kp_feet,                    dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(kd_feet,                    dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(kp_hands,                   dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(kd_hands,                   dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(kp_waist,                   dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(kd_waist,                   dynamicgraph::Vector);


  // DECLARE_SIGNAL_IN(w_feet,                     double);
  // DECLARE_SIGNAL_IN(w_waist,                    double);
  // DECLARE_SIGNAL_IN(w_torques,                  double);
  // DECLARE_SIGNAL_IN(w_forces,                   double);

  // DECLARE_SIGNAL_IN(mu,                         double);
  // DECLARE_SIGNAL_IN(contact_points,             dynamicgraph::Matrix);
  // DECLARE_SIGNAL_IN(contact_normal,             dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(f_min,                      double);
  // DECLARE_SIGNAL_IN(f_max_right_foot,           double);
  // DECLARE_SIGNAL_IN(f_max_left_foot,            double);

  // DECLARE_SIGNAL_IN(tau_max,                    dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(q_min,                      dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(q_max,                      dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(dq_max,                     dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(ddq_max,                    dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(dt_joint_pos_limits,        double);

  // DECLARE_SIGNAL_IN(tau_estimated,              dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(wrench_base,                dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(wrench_left_foot,           dynamicgraph::Vector);
  // DECLARE_SIGNAL_IN(wrench_right_foot,          dynamicgraph::Vector);

  /// This signal copies active_joints only if it changes from a all false or to an all false value


  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display( std::ostream& os ) const;

  void sendMsg(const std::string& msg, MsgType t = MSG_TYPE_INFO, const char* file = "", int line = 0) {
    Entity::sendMsg("[" + name + "] " + msg, t, file, line);
  }

 protected:

  double            m_dt;               /// control loop time period
  double            m_t;
  bool              m_initSucceeded;    /// true if the entity has been successfully initialized
  bool              m_enabled;          /// True if controler is enabled
  bool              m_firstTime;        /// True at the first iteration of the controller

  // int m_frame_id_rf;  /// frame id of right foot
  // int m_frame_id_lf;  /// frame id of left foot

  /// tsid
  tsid::robots::RobotWrapper *             m_robot;
  tsid::solvers::SolverHQPBase *           m_hqpSolver;

  tsid::InverseDynamicsFormulationAccForce * m_invDyn;
  // tsid::contacts::Contact6d *                m_contactRF;
  // tsid::contacts::Contact6d *                m_contactLF;
  tsid::tasks::TaskComEquality *             m_taskCom;
  // tsid::tasks::TaskSE3Equality *             m_taskWaist;
  tsid::tasks::TaskJointPosture *            m_taskPosture;
  tsid::tasks::TaskJointPosture *            m_taskBlockedJoints;

  tsid::trajectories::TrajectorySample       m_sampleCom;
  // tsid::trajectories::TrajectorySample       m_sampleWaist;
  tsid::trajectories::TrajectorySample       m_samplePosture;

  double m_w_com;
  double m_w_posture;
  // double m_w_waist;

  tsid::math::Vector  m_dv_sot;              /// desired accelerations (sot order)
  tsid::math::Vector  m_dv_urdf;             /// desired accelerations (urdf order)
  tsid::math::Vector  m_v_sot;
  tsid::math::Vector  m_q_sot;
  // tsid::math::Vector  m_f;                   /// desired force coefficients (24d)
  // tsid::math::Vector6 m_f_RF;                /// desired 6d wrench right foot
  // tsid::math::Vector6 m_f_LF;                /// desired 6d wrench left foot
  tsid::math::Vector3 m_com_offset;          /// 3d CoM offset
  tsid::math::Vector  m_tau_sot;
  tsid::math::Vector  m_q_urdf;
  tsid::math::Vector  m_v_urdf;

  // typedef pinocchio::Data::Matrix6x Matrix6x;
  // Matrix6x m_J_RF;
  // Matrix6x m_J_LF;
  // Eigen::ColPivHouseholderQR<Matrix6x> m_J_RF_QR;
  // Eigen::ColPivHouseholderQR<Matrix6x> m_J_LF_QR;
  // tsid::math::Vector6 m_v_RF_int;
  // tsid::math::Vector6 m_v_LF_int;

  unsigned int m_timeLast;
  RobotUtilShrPtr m_robot_util;

}; // class SimpleInverseDyn
}    // namespace torque_control
}      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_simple_inverse_dyn_H__
