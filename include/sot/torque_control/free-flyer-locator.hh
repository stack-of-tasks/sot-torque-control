/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_free_flyer_locator_H__
#define __sot_torque_control_free_flyer_locator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(free_flyer_locator_EXPORTS)
#define SOTFREEFLYERLOCATOR_EXPORT __declspec(dllexport)
#else
#define SOTFREEFLYERLOCATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTFREEFLYERLOCATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <map>
#include "boost/assign.hpp"

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
//~ #include <pinocchio/algorithm/rnea.hpp>
//~ #include "pinocchio/algorithm/crba.hpp"

#include <dynamic-graph/linear-algebra.h>
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

class SOTFREEFLYERLOCATOR_EXPORT FreeFlyerLocator : public ::dynamicgraph::Entity {
  typedef FreeFlyerLocator EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  FreeFlyerLocator(const std::string& name);
  ~FreeFlyerLocator();

  void init(const std::string& robotRef);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(base6d_encoders, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(joint_velocities, dynamicgraph::Vector);
  DECLARE_SIGNAL_INNER(kinematics_computations, dynamicgraph::Vector);
  /// freeflyer position with angle axis format
  DECLARE_SIGNAL_OUT(freeflyer_aa, dynamicgraph::Vector);
  /// base6d_encoders with base6d in RPY
  DECLARE_SIGNAL_OUT(base6dFromFoot_encoders, dynamicgraph::Vector);
  /// n+6 robot velocities
  DECLARE_SIGNAL_OUT(v, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  void displayRobotUtil();

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool m_initSucceeded;       /// true if the entity has been successfully initialized
  pinocchio::Model* m_model;  /// Pinocchio robot model
  pinocchio::Data* m_data;    /// Pinocchio robot data
  pinocchio::SE3 m_Mff;       /// SE3 Transform from center of feet to base
  pinocchio::SE3 m_w_M_lf;
  pinocchio::SE3 m_w_M_rf;
  long unsigned int m_right_foot_id;
  long unsigned int m_left_foot_id;
  Eigen::VectorXd m_q_pin;  /// robot configuration according to pinocchio convention
  Eigen::VectorXd m_q_sot;  /// robot configuration according to SoT convention
  Eigen::VectorXd m_v_pin;  /// robot velocities according to pinocchio convention
  Eigen::VectorXd m_v_sot;  /// robot velocities according to SoT convention

  RobotUtilShrPtr m_robot_util;

};  // class FreeFlyerLocator

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_free_flyer_locator_H__
