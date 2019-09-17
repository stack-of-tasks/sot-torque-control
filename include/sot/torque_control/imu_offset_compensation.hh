/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 */

#ifndef __sot_torque_control_imu_offset_compensation_H__
#define __sot_torque_control_imu_offset_compensation_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(imu_offset_compensation_EXPORTS)
#define SOTIMUOFFSETCOMPENSATION_EXPORT __declspec(dllexport)
#else
#define SOTIMUOFFSETCOMPENSATION_EXPORT __declspec(dllimport)
#endif
#else
#define SOTIMUOFFSETCOMPENSATION_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <map>
#include "boost/assign.hpp"

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

class SOTIMUOFFSETCOMPENSATION_EXPORT ImuOffsetCompensation : public ::dynamicgraph::Entity {
  typedef ImuOffsetCompensation EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();
  typedef Eigen::Vector3d Vector3;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  ImuOffsetCompensation(const std::string& name);

  /* --- COMMANDS --- */
  void init(const double& dt);
  void update_offset(const double& duration);
  void setGyroDCBlockerParameter(const double& alpha);
  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(accelerometer_in, dynamicgraph::Vector);    /// raw accelerometer data
  DECLARE_SIGNAL_IN(gyrometer_in, dynamicgraph::Vector);        /// raw gyrometer data
  DECLARE_SIGNAL_OUT(accelerometer_out, dynamicgraph::Vector);  /// compensated accelerometer data
  DECLARE_SIGNAL_OUT(gyrometer_out, dynamicgraph::Vector);      /// compensated gyrometer data

 protected:
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

  /* --- METHODS --- */
  void update_offset_impl(int iter);
  void sendMsg(const std::string& msg, MsgType t = MSG_TYPE_INFO, const char* file = "", int line = 0) {
    Entity::sendMsg("[ImuOffsetCompensation-" + name + "] " + msg, t, file, line);
  }

 protected:
  bool m_initSucceeded;        /// true if the entity has been successfully initialized
  float m_dt;                  /// sampling time in seconds
  int m_update_cycles_left;    /// number of update cycles left
  int m_update_cycles;         /// total number of update cycles to perform
  double m_a_gyro_DC_blocker;  /// filter parameter to remove DC from gyro online (should be close to <1.0 and equal
                               /// to 1.0 for disabling)
  Vector3 m_gyro_offset;       /// gyrometer offset
  Vector3 m_acc_offset;        /// accelerometer offset

  Vector3 m_gyro_sum;  /// tmp variable to store the sum of the gyro measurements during update phase
  Vector3 m_acc_sum;   /// tmp variable to store the sum of the acc measurements during update phase

};  // class ImuOffsetCompensation
}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_imu_offset_compensation_H__
