/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_JointTorqueController_H__
#define __sot_torque_control_JointTorqueController_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(joint_torque_controller_EXPORTS)
#define SOTJOINTTORQUECONTROLLER_EXPORT __declspec(dllexport)
#else
#define SOTJOINTTORQUECONTROLLER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTJOINTTORQUECONTROLLER_EXPORT
#endif

// #define VP_DEBUG 1        /// enable debug output
// #define VP_DEBUG_MODE 20

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <pinocchio/fwd.hpp>
#include <tsid/utils/stop-watch.hpp>
/* HELPER */
#include <dynamic-graph/signal-helper.h>

#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <sot/torque_control/utils/vector-conversions.hh>

/*Motor model*/
#include <sot/torque_control/motor-model.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

/**
 * This Entity takes as inputs the estimated joints' positions,
 * velocities, accelerations and torques and it computes the desired
 * current to send to the motors board in order to track the
 * desired joints torques. Most of the input of this entity are
 * computed by the entity ForceTorqueEstimator.
 *
 * QUICK START
 * Create the entity, plug all the input signals, call the init method
 * specifying the control-loop time step. For instance:
 *   jtc = JointTorqueController("jtc");
 *   plug(estimator.jointsPositions,     jtc.jointsPositions);
 *   plug(estimator.jointsVelocities,    jtc.jointsVelocities);
 *   plug(estimator.jointsAccelerations, jtc.jointsAccelerations);
 *   plug(estimator.jointsTorques,       jtc.jointsTorques);
 *   jtc.KpTorque.value = N_DOF*(10.0,);
 *   jtc.KiTorque.value = N_DOF*(0.01,);
 *   jtc.init(dt);
 *
 * DETAILS
 * To do...
 */
class SOTJOINTTORQUECONTROLLER_EXPORT JointTorqueController
    : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public: /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(jointsPositions, dynamicgraph::Vector);      /// q
  DECLARE_SIGNAL_IN(jointsVelocities, dynamicgraph::Vector);     /// dq
  DECLARE_SIGNAL_IN(jointsAccelerations, dynamicgraph::Vector);  /// ddq
  DECLARE_SIGNAL_IN(jointsTorques,
                    dynamicgraph::Vector);  /// estimated joints torques tau
  DECLARE_SIGNAL_IN(
      jointsTorquesDerivative,
      dynamicgraph::Vector);  /// estimated joints torques derivative dtau
  DECLARE_SIGNAL_IN(jointsTorquesDesired,
                    dynamicgraph::Vector);  /// desired joints torques tauDes
  //        DECLARE_SIGNAL_IN(jointsTorquesDesiredDerivative,
  //        dynamicgraph::Vector);/// desired joints torques derivative dtauDes
  DECLARE_SIGNAL_IN(dq_des, dynamicgraph::Vector);  /// desired joint velocities
  DECLARE_SIGNAL_IN(KpTorque,
                    dynamicgraph::Vector);  /// proportional gain for torque
                                            /// feedback controller
  DECLARE_SIGNAL_IN(
      KiTorque,
      dynamicgraph::Vector);  /// integral gain for torque feedback controller
  DECLARE_SIGNAL_IN(
      KdTorque,
      dynamicgraph::Vector);  /// derivative gain for torque feedback controller
  DECLARE_SIGNAL_IN(
      KdVel, dynamicgraph::Vector);  /// derivative gain for velocity feedback
  DECLARE_SIGNAL_IN(
      KiVel, dynamicgraph::Vector);  /// integral gain for velocity feedback
  DECLARE_SIGNAL_IN(torque_integral_saturation,
                    dynamicgraph::Vector);  /// integral error saturation

  //        DECLARE_SIGNAL_IN(dq_threshold,           dynamicgraph::Vector); ///
  //        velocity sign threshold DECLARE_SIGNAL_IN(ddq_threshold,
  //        dynamicgraph::Vector);      /// acceleration sign threshold

  /// parameters for the linear model
  DECLARE_SIGNAL_IN(coulomb_friction_compensation_percentage,
                    dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKt_p, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKt_n, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKf_p, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKf_n, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKv_p, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKv_n, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKa_p, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(motorParameterKa_n, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(polySignDq, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(u, dynamicgraph::Vector);  /// Desired current
  DECLARE_SIGNAL_OUT(
      smoothSignDq, dynamicgraph::Vector);  /// smooth approximation of sign(dq)
  DECLARE_SIGNAL_OUT(
      torque_error_integral,
      dynamicgraph::Vector);  /// integral of the torque tracking error

 protected:
  MotorModel motorModel;
  double m_dt;  /// timestep of the controller
  Eigen::VectorXd m_tau_star;
  Eigen::VectorXd m_current_des;
  Eigen::VectorXd m_tauErrIntegral;      /// integral of the torque error
  Eigen::VectorXd m_currentErrIntegral;  /// integral of the current error
  //        Eigen::VectorXd m_dqDesIntegral; /// integral of the desired
  //        velocity
  Eigen::VectorXd m_dqErrIntegral;  /// integral of the velocity error

  RobotUtilShrPtr m_robot_util;

  void sendMsg(const std::string& msg, MsgType t = MSG_TYPE_INFO,
               const char* = "", int = 0) {
    logger_.stream(t) << ("[" + name + "] " + msg) << '\n';
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** --- CONSTRUCTOR ---- */
  JointTorqueController(const std::string& name);

  /** Initialize the JointTorqueController.
   * @param timestep Period (in seconds) after which the sensors' data are
   * updated.
   */
  void init(const double& timestep, const std::string& robotRef);

  void reset_integral();

 public: /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

};  // class JointTorqueController

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_JointTorqueController_H__
