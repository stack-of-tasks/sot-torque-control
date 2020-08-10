/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_inverse_dynamics_balance_controller_H__
#define __sot_torque_control_inverse_dynamics_balance_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(inverse_dynamics_balance_controller_EXPORTS)
#define SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT __declspec(dllexport)
#else
#define SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT
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
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <map>
#include "boost/assign.hpp"

#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-angular-momentum-equality.hpp>
// #include "tsid/tasks/task-angular-momentum-integral-equality.hpp"
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

class SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT InverseDynamicsBalanceController : public ::dynamicgraph::Entity {
  typedef InverseDynamicsBalanceController EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  InverseDynamicsBalanceController(const std::string& name);

  void init(const double& dt, const std::string& robotRef);
  void updateComOffset();
  virtual void setControlOutputType(const std::string& type);
  void removeRightFootContact(const double& transitionTime);
  void removeLeftFootContact(const double& transitionTime);
  void addRightFootContact(const double& transitionTime);
  void addLeftFootContact(const double& transitionTime);
  void addTaskRightHand(/*const double& transitionTime*/);
  void removeTaskRightHand(const double& transitionTime);
  void addTaskLeftHand(/*const double& transitionTime*/);
  void removeTaskLeftHand(const double& transitionTime);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(com_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_adm_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_adm_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(am_ref_L, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(am_ref_dL, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rf_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rf_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rf_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(lf_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(lf_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(lf_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rh_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rh_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rh_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(lh_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(lh_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(lh_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(posture_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(base_orientation_ref_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(base_orientation_ref_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(base_orientation_ref_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(f_ref_right_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(f_ref_left_foot, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(kp_base_orientation, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_base_orientation, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_constraints, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_constraints, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_com, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_com, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_am, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_am, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_feet, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_feet, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_hands, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_hands, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_posture, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_posture, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kd_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_tau, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(w_com, double);
  DECLARE_SIGNAL_IN(w_am, double);
  DECLARE_SIGNAL_IN(w_feet, double);
  DECLARE_SIGNAL_IN(w_hands, double);
  DECLARE_SIGNAL_IN(w_posture, double);
  DECLARE_SIGNAL_IN(w_base_orientation, double);
  DECLARE_SIGNAL_IN(w_torques, double);
  DECLARE_SIGNAL_IN(w_forces, double);
  DECLARE_SIGNAL_IN(weight_contact_forces, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(mu, double);
  DECLARE_SIGNAL_IN(contact_points, dynamicgraph::Matrix);
  DECLARE_SIGNAL_IN(contact_normal, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(f_min, double);
  DECLARE_SIGNAL_IN(f_max_right_foot, double);
  DECLARE_SIGNAL_IN(f_max_left_foot, double);

  DECLARE_SIGNAL_IN(rotor_inertias, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(gear_ratios, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(tau_max, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(q_min, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(q_max, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(dq_max, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(ddq_max, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(dt_joint_pos_limits, double);

  DECLARE_SIGNAL_IN(tau_measured, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(com_measured, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(q, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(v, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrench_base, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrench_left_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(wrench_right_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(ref_phase, int);

  DECLARE_SIGNAL_IN(active_joints, dynamicgraph::Vector);  /// mask with 1 for controlled joints, 0 otherwise

  DECLARE_SIGNAL_OUT(tau_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(M, dynamicgraph::Matrix);
  DECLARE_SIGNAL_OUT(dv_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(v_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(q_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(tau_pd_des, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(f_des_right_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(f_des_left_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_des_right_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_des_left_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_des_right_foot_local, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_des_left_foot_local, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_ref, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_right_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp_left_foot, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(zmp, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(com, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(com_est, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(com_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(com_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(com_acc_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(am_dL, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(am_dL_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(base_orientation, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_pos_quat, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_pos_quat, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(lf_est, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(rf_est, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_hand_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_hand_pos, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_hand_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_hand_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_hand_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_hand_acc, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(right_foot_acc_des, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(left_foot_acc_des, dynamicgraph::Vector);

  /// This signal copies active_joints only if it changes from a all false or to an all false value
  DECLARE_SIGNAL_INNER(active_joints_checked, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  double m_dt;  /// control loop time period
  double m_t;
  bool m_initSucceeded;  /// true if the entity has been successfully initialized
  bool m_enabled;        /// True if controler is enabled
  bool m_firstTime;      /// True at the first iteration of the controller

  enum ContactState {
    DOUBLE_SUPPORT = 0,
    LEFT_SUPPORT = 1,
    LEFT_SUPPORT_TRANSITION = 2,  // transition towards left support
    RIGHT_SUPPORT = 3,
    RIGHT_SUPPORT_TRANSITION = 4
  };
  ContactState m_contactState;
  double m_contactTransitionTime;  /// end time of the current contact transition (if any)

  enum RightHandState {
    TASK_RIGHT_HAND_ON = 0,
    /*TASK_RIGHT_HAND_TRANSITION = 1,*/
    TASK_RIGHT_HAND_OFF = 1
  };
  RightHandState m_rightHandState;

  enum LeftHandState {
    TASK_LEFT_HAND_ON = 0,
    /*TASK_LEFT_HAND_TRANSITION = 1,*/
    TASK_LEFT_HAND_OFF = 1
  };
  LeftHandState m_leftHandState;
  /*double            m_handsTransitionTime;*/  /// end time of the current transition (if any)

  int m_frame_id_rf;  /// frame id of right foot
  int m_frame_id_lf;  /// frame id of left foot

  int m_frame_id_rh;  /// frame id of right hand
  int m_frame_id_lh;  /// frame id of left hand

  /// tsid
  tsid::robots::RobotWrapper* m_robot;
  tsid::solvers::SolverHQPBase* m_hqpSolver;
  tsid::solvers::SolverHQPBase* m_hqpSolver_60_36_34;
  tsid::solvers::SolverHQPBase* m_hqpSolver_48_30_17;
  tsid::InverseDynamicsFormulationAccForce* m_invDyn;
  tsid::contacts::Contact6d* m_contactRF;
  tsid::contacts::Contact6d* m_contactLF;
  tsid::contacts::Contact6d* m_contactRH;
  tsid::contacts::Contact6d* m_contactLH;
  tsid::tasks::TaskComEquality* m_taskCom;
  tsid::tasks::TaskComEquality* m_taskComAdm;
  tsid::tasks::TaskAMEquality* m_taskAM;
  tsid::tasks::TaskSE3Equality* m_taskWaist;
  tsid::tasks::TaskSE3Equality* m_taskRF;
  tsid::tasks::TaskSE3Equality* m_taskLF;
  tsid::tasks::TaskSE3Equality* m_taskRH;
  tsid::tasks::TaskSE3Equality* m_taskLH;
  tsid::tasks::TaskJointPosture* m_taskPosture;
  tsid::tasks::TaskJointPosture* m_taskBlockedJoints;

  tsid::trajectories::TrajectorySample m_sampleCom;
  tsid::trajectories::TrajectorySample m_sampleComAdm;
  tsid::trajectories::TrajectorySample m_sampleAM;
  tsid::trajectories::TrajectorySample m_sampleRF;
  tsid::trajectories::TrajectorySample m_sampleLF;
  tsid::trajectories::TrajectorySample m_sampleRH;
  tsid::trajectories::TrajectorySample m_sampleLH;
  tsid::trajectories::TrajectorySample m_sampleWaist;
  tsid::trajectories::TrajectorySample m_samplePosture;

  double m_w_com;
  double m_w_am;
  double m_w_posture;
  double m_w_hands;
  double m_w_base_orientation;

  tsid::math::Vector m_dv_sot;    /// desired accelerations (sot order)
  tsid::math::Vector m_dv_urdf;   /// desired accelerations (urdf order)
  tsid::math::Vector m_v_sot;     /// desired velocities (sot order)
  tsid::math::Vector m_v_urdf;    /// desired and current velocities (urdf order) (close the TSID loop on it)
  tsid::math::Vector m_q_sot;     /// desired positions (sot order)
  tsid::math::Vector m_q_urdf;    /// desired and current positions (urdf order) (close the TSID loop on it)
  tsid::math::Vector m_tau_sot;   /// desired torques (sot order)

  tsid::math::Vector m_f;                  /// desired force coefficients (24d)
  tsid::math::Vector6 m_f_RF;              /// desired 6d wrench right foot
  tsid::math::Vector6 m_f_LF;              /// desired 6d wrench left foot
  tsid::math::Vector3 m_com_offset;        /// 3d CoM offset
  tsid::math::Vector3 m_dcom_offset;        /// 3d CoM offset
  tsid::math::Vector3 m_zmp_des_LF;        /// 3d desired zmp left foot
  tsid::math::Vector3 m_zmp_des_RF;        /// 3d desired zmp left foot
  tsid::math::Vector3 m_zmp_des_LF_local;  /// 3d desired zmp left foot expressed in local frame
  tsid::math::Vector3 m_zmp_des_RF_local;  /// 3d desired zmp left foot expressed in local frame
  tsid::math::Vector3 m_zmp_des;           /// 3d desired global zmp
  tsid::math::Vector3 m_zmp_LF;            /// 3d zmp left foot
  tsid::math::Vector3 m_zmp_RF;            /// 3d zmp left foot
  tsid::math::Vector3 m_zmp;               /// 3d global zmp


  typedef pinocchio::Data::Matrix6x Matrix6x;
  pinocchio::Data m_estim_data;
  Matrix6x m_J_RF;
  Matrix6x m_J_LF;
  Eigen::ColPivHouseholderQR<Matrix6x> m_J_RF_QR;
  Eigen::ColPivHouseholderQR<Matrix6x> m_J_LF_QR;
  tsid::math::Vector6 m_v_RF_int;
  tsid::math::Vector6 m_v_LF_int;

  unsigned int m_timeLast;        /// Final time of the control loop
  RobotUtilShrPtr m_robot_util;   /// Share pointer to the robot utils methods
  ControlOutput m_ctrlMode;       /// ctrl mode desired for the output (velocity or torque)

};  // class InverseDynamicsBalanceController
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_inverse_dynamics_balance_controller_H__
