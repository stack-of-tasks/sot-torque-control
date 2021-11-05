/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_admittance_controller_H__
#define __sot_torque_control_admittance_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(sot_admittance_controller_EXPORTS)
#define SOTADMITTANCECONTROLLER_EXPORT __declspec(dllexport)
#else
#define SOTADMITTANCECONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTADMITTANCECONTROLLER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <map>
#include "boost/assign.hpp"

#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/tasks/task-se3-equality.hpp>

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

class SOTADMITTANCECONTROLLER_EXPORT AdmittanceController : public ::dynamicgraph::Entity {
  typedef AdmittanceController EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  AdmittanceController(const std::string& name);

  void init(const double& dt, const std::string& robotRef);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(encoders, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(jointsVelocities, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_force, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(ki_force, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(kp_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(ki_vel, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(force_integral_saturation, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(force_integral_deadzone, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(fRightFootRef, dynamicgraph::Vector);       /// 6d reference force
  DECLARE_SIGNAL_IN(fLeftFootRef, dynamicgraph::Vector);        /// 6d reference force
  DECLARE_SIGNAL_IN(fRightFoot, dynamicgraph::Vector);          /// 6d estimated force
  DECLARE_SIGNAL_IN(fLeftFoot, dynamicgraph::Vector);           /// 6d estimated force
  DECLARE_SIGNAL_IN(fRightFootFiltered, dynamicgraph::Vector);  /// 6d estimated force filtered
  DECLARE_SIGNAL_IN(fLeftFootFiltered, dynamicgraph::Vector);   /// 6d estimated force filtered
  DECLARE_SIGNAL_IN(controlledJoints, dynamicgraph::Vector);    /// mask with 1 for controlled joints, 0 otherwise
  DECLARE_SIGNAL_IN(damping, dynamicgraph::Vector);             /// damping factors used for the 4 end-effectors

  //        DECLARE_SIGNAL_IN(fRightHandRef,    dynamicgraph::Vector); /// 6d reference force
  //        DECLARE_SIGNAL_IN(fLeftHandRef,     dynamicgraph::Vector); /// 6d reference force
  //        DECLARE_SIGNAL_IN(fRightHand,       dynamicgraph::Vector); /// 6d estimated force
  //        DECLARE_SIGNAL_IN(fLeftHand,        dynamicgraph::Vector); /// 6d estimated force

  DECLARE_SIGNAL_OUT(u, dynamicgraph::Vector);  /// control
  // DEBUG SIGNALS
  DECLARE_SIGNAL_OUT(dqDes, dynamicgraph::Vector);          /// dqDes = J^+ * Kf * (fRef-f)
  DECLARE_SIGNAL_OUT(vDesRightFoot, dynamicgraph::Vector);  ///
  DECLARE_SIGNAL_OUT(vDesLeftFoot, dynamicgraph::Vector);   ///
  //        DECLARE_SIGNAL_OUT(fRightHandError,   dynamicgraph::Vector);  /// fRef-f
  //        DECLARE_SIGNAL_OUT(fLeftHandError,    dynamicgraph::Vector);  /// fRef-f

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  Eigen::VectorXd m_u;  /// control (i.e. motor currents)
  bool m_firstIter;
  bool m_initSucceeded;         /// true if the entity has been successfully initialized
  bool m_useJacobianTranspose;  /// if true it uses the Jacobian transpose rather than the pseudoinverse
  double m_dt;                  /// control loop time period
  int m_nj;

  /// robot geometric/inertial data
  int m_frame_id_rf;  /// frame id of right foot
  int m_frame_id_lf;  /// frame id of left foot

  /// tsid
  tsid::robots::RobotWrapper* m_robot;
  pinocchio::Data* m_data;

  tsid::math::Vector6 m_f_RF;  /// desired 6d wrench right foot
  tsid::math::Vector6 m_f_LF;  /// desired 6d wrench left foot
  tsid::math::Vector m_q_urdf;
  tsid::math::Vector m_v_urdf;

  tsid::math::Vector m_dq_des_urdf;
  tsid::math::Vector m_dqErrIntegral;  /// integral of the velocity error
  //        tsid::math::Vector  m_dqDesIntegral;    /// integral of the desired joint velocities
  tsid::math::Vector m_dq_fd;  /// joint velocities computed with finite differences
  tsid::math::Vector m_qPrev;  /// previous value of encoders

  typedef pinocchio::Data::Matrix6x Matrix6x;
  Matrix6x m_J_RF;
  Matrix6x m_J_LF;
  Eigen::ColPivHouseholderQR<Matrix6x> m_J_RF_QR;
  Eigen::ColPivHouseholderQR<Matrix6x> m_J_LF_QR;
  tsid::math::Vector6 m_v_RF_int;
  tsid::math::Vector6 m_v_LF_int;

  RobotUtilShrPtr m_robot_util;

  //        tsid::math::Vector3 m_zmp_des_LF;          /// 3d desired zmp left foot
  //        tsid::math::Vector3 m_zmp_des_RF;          /// 3d desired zmp left foot
  //        tsid::math::Vector3 m_zmp_des_LF_local;    /// 3d desired zmp left foot expressed in local frame
  //        tsid::math::Vector3 m_zmp_des_RF_local;    /// 3d desired zmp left foot expressed in local frame
  //        tsid::math::Vector3 m_zmp_des;             /// 3d desired global zmp
  //        tsid::math::Vector3 m_zmp_LF;              /// 3d zmp left foot
  //        tsid::math::Vector3 m_zmp_RF;              /// 3d zmp left foot
  //        tsid::math::Vector3 m_zmp;                 /// 3d global zmp
};  // class AdmittanceController

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_admittance_controller_H__
