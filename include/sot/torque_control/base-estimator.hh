/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_base_estimator_H__
#define __sot_torque_control_base_estimator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(base_estimator_EXPORTS)
#define SOTBASEESTIMATOR_EXPORT __declspec(dllexport)
#else
#define SOTBASEESTIMATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTBASEESTIMATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <pinocchio/spatial/fwd.hpp>
#include <sot/core/robot-utils.hh>
#include <map>
#include "boost/assign.hpp"
//#include <boost/random/normal_distribution.hpp>
#include <boost/math/distributions/normal.hpp>  // for normal_distribution

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/torque_control/utils/vector-conversions.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/** Compute s12 as an intermediate transform between s1 and s2 SE3 transforms**/
void se3Interp(const pinocchio::SE3& s1, const pinocchio::SE3& s2, const double alpha, pinocchio::SE3& s12);

/** Convert from Roll, Pitch, Yaw to transformation Matrix. */
void rpyToMatrix(double r, double p, double y, Eigen::Matrix3d& R);

/** Convert from Roll, Pitch, Yaw to transformation Matrix. */
void rpyToMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R);

/**  Convert from Transformation Matrix to Roll, Pitch, Yaw */
void matrixToRpy(const Eigen::Matrix3d& M, Eigen::Vector3d& rpy);

/** Multiply to quaternions stored in (w,x,y,z) format */
void quanternionMult(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2, Eigen::Vector4d& q12);

/** Rotate a point or a vector by a quaternion stored in (w,x,y,z) format */
void pointRotationByQuaternion(const Eigen::Vector3d& point, const Eigen::Vector4d& quat,
                               Eigen::Vector3d& rotatedPoint);

class SOTBASEESTIMATOR_EXPORT BaseEstimator : public ::dynamicgraph::Entity {
  typedef BaseEstimator EntityClassName;
  typedef pinocchio::SE3 SE3;
  typedef Eigen::Vector2d Vector2;
  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Vector4d Vector4;
  typedef dynamicgraph::sot::Vector6d Vector6;
  typedef dynamicgraph::sot::Vector7d Vector7;
  typedef Eigen::Matrix3d Matrix3;
  typedef boost::math::normal normal;

  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  BaseEstimator(const std::string& name);

  void init(const double& dt, const std::string& urdfFile);

  void set_fz_stable_windows_size(const int& ws);
  void set_alpha_w_filter(const double& a);
  void set_alpha_DC_acc(const double& a);
  void set_alpha_DC_vel(const double& a);
  void reset_foot_positions();
  void set_imu_weight(const double& w);
  void set_zmp_std_dev_right_foot(const double& std_dev);
  void set_zmp_std_dev_left_foot(const double& std_dev);
  void set_normal_force_std_dev_right_foot(const double& std_dev);
  void set_normal_force_std_dev_left_foot(const double& std_dev);
  void set_stiffness_right_foot(const dynamicgraph::Vector& k);
  void set_stiffness_left_foot(const dynamicgraph::Vector& k);
  void set_right_foot_sizes(const dynamicgraph::Vector& s);
  void set_left_foot_sizes(const dynamicgraph::Vector& s);
  void set_zmp_margin_right_foot(const double& margin);
  void set_zmp_margin_left_foot(const double& margin);
  void set_normal_force_margin_right_foot(const double& margin);
  void set_normal_force_margin_left_foot(const double& margin);

  void reset_foot_positions_impl(const Vector6& ftlf, const Vector6& ftrf);
  void compute_zmp(const Vector6& w, Vector2& zmp);
  double compute_zmp_weight(const Vector2& zmp, const Vector4& foot_sizes, double std_dev, double margin);
  double compute_force_weight(double fz, double std_dev, double margin);
  void kinematics_estimation(const Vector6& ft, const Vector6& K, const SE3& oMfs, const int foot_id, SE3& oMff,
                             SE3& oMfa, SE3& fsMff);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(joint_positions, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(joint_velocities, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(imu_quaternion, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(forceLLEG, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(forceRLEG, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(dforceLLEG, dynamicgraph::Vector);  /// derivative of left force torque sensor
  DECLARE_SIGNAL_IN(dforceRLEG, dynamicgraph::Vector);  /// derivative of right force torque sensor
  DECLARE_SIGNAL_IN(w_lf_in, double);                   /// weight of the estimation coming from the left foot
  DECLARE_SIGNAL_IN(w_rf_in, double);                   /// weight of the estimation coming from the right foot
  DECLARE_SIGNAL_IN(
      K_fb_feet_poses,
      double);  /// feed back gain to correct feet position according to last base estimation and kinematic
  DECLARE_SIGNAL_IN(lf_ref_xyzquat, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(rf_ref_xyzquat, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(accelerometer, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(gyroscope, dynamicgraph::Vector);

  DECLARE_SIGNAL_INNER(kinematics_computations, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(q, dynamicgraph::Vector);      /// n+6 robot configuration with base6d in RPY
  DECLARE_SIGNAL_OUT(v, dynamicgraph::Vector);      /// n+6 robot velocities
  DECLARE_SIGNAL_OUT(v_kin, dynamicgraph::Vector);  /// 6d robot velocities from kinematic only   (encoders derivative)
  DECLARE_SIGNAL_OUT(v_flex,
                     dynamicgraph::Vector);  /// 6d robot velocities from flexibility only (force sensor derivative)
  DECLARE_SIGNAL_OUT(v_imu,
                     dynamicgraph::Vector);  /// 6d robot velocities form imu only (accelerometer integration + gyro)
  DECLARE_SIGNAL_OUT(v_gyr, dynamicgraph::Vector);  /// 6d robot velocities form gyroscope only (as if gyro measured
                                                    /// the pure angular ankle velocities)
  DECLARE_SIGNAL_OUT(lf_xyzquat, dynamicgraph::Vector);  /// left foot pose
  DECLARE_SIGNAL_OUT(rf_xyzquat, dynamicgraph::Vector);  /// right foot pose
  DECLARE_SIGNAL_OUT(a_ac, dynamicgraph::Vector);  /// acceleration of the base in the world with DC component removed
  DECLARE_SIGNAL_OUT(v_ac, dynamicgraph::Vector);  /// velocity of the base in the world with DC component removed

  DECLARE_SIGNAL_OUT(q_lf, dynamicgraph::Vector);   /// n+6 robot configuration with base6d in RPY
  DECLARE_SIGNAL_OUT(q_rf, dynamicgraph::Vector);   /// n+6 robot configuration with base6d in RPY
  DECLARE_SIGNAL_OUT(q_imu, dynamicgraph::Vector);  /// n+6 robot configuration with base6d in RPY
  DECLARE_SIGNAL_OUT(w_lf, double);                 /// weight of the estimation coming from the left foot
  DECLARE_SIGNAL_OUT(w_rf, double);                 /// weight of the estimation coming from the right foot
  DECLARE_SIGNAL_OUT(w_lf_filtered, double);        /// filtered weight of the estimation coming from the left foot
  DECLARE_SIGNAL_OUT(w_rf_filtered, double);        /// filtered weight of the estimation coming from the right foot

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool m_initSucceeded;   /// true if the entity has been successfully initialized
  bool m_reset_foot_pos;  /// true after the command resetFootPositions is called
  double m_dt;            /// sampling time step
  RobotUtilShrPtr m_robot_util;

  bool m_left_foot_is_stable;    /// True if left foot as been stable for the last 'm_fz_stable_windows_size' samples
  bool m_right_foot_is_stable;   /// True if right foot as been stable for the last 'm_fz_stable_windows_size' samples
  int m_fz_stable_windows_size;  /// size of the windows used to detect that feet did not leave the ground
  int m_lf_fz_stable_cpt;        /// counter for detecting for how long the feet has been stable
  int m_rf_fz_stable_cpt;        /// counter for detecting for how long the feet has been stable

  /* Estimator parameters */
  double m_w_imu;              /// weight of IMU for sensor fusion
  double m_zmp_std_dev_rf;     /// standard deviation of ZMP measurement errors for right foot
  double m_zmp_std_dev_lf;     /// standard deviation of ZMP measurement errors for left foot
  double m_fz_std_dev_rf;      /// standard deviation of normal force measurement errors for right foot
  double m_fz_std_dev_lf;      /// standard deviation of normal force measurement errors for left foot
  Vector4 m_left_foot_sizes;   /// sizes of the left foot (pos x, neg x, pos y, neg y)
  Vector4 m_right_foot_sizes;  /// sizes of the left foot (pos x, neg x, pos y, neg y)
  double m_zmp_margin_lf;      /// margin used for computing zmp weight
  double m_zmp_margin_rf;      /// margin used for computing zmp weight
  double m_fz_margin_lf;       /// margin used for computing normal force weight
  double m_fz_margin_rf;       /// margin used for computing normal force weight
  Vector6 m_K_rf;              /// 6d stiffness of right foot spring
  Vector6 m_K_lf;              /// 6d stiffness of left foot spring

  Eigen::VectorXd m_v_kin;   /// 6d robot velocities from kinematic only   (encoders derivative)
  Eigen::VectorXd m_v_flex;  /// 6d robot velocities from flexibility only (force sensor derivative)
  Eigen::VectorXd m_v_imu;   /// 6d robot velocities form imu only (accelerometer integration + gyro)
  Eigen::VectorXd
      m_v_gyr;  /// 6d robot velocities form gyroscope only (as if gyro measured the pure angular ankle velocities)

  Vector3 m_v_ac;  /// velocity of the base in the world with DC component removed
  Vector3 m_a_ac;  /// acceleration of the base in the world with DC component removed

  double m_alpha_DC_acc;  /// alpha parameter for DC blocker filter on acceleration data
  double m_alpha_DC_vel;  /// alpha parameter for DC blocker filter on velocity data

  double m_alpha_w_filter;  /// filter parameter to filter weights (1st order low pass filter)

  double m_w_lf_filtered;  /// filtered weight of the estimation coming from the left foot
  double m_w_rf_filtered;  /// filtered weight of the estimation coming from the right foot

  pinocchio::Model m_model;  /// Pinocchio robot model
  pinocchio::Data* m_data;   /// Pinocchio robot data
  pinocchio::SE3 m_oMff_lf;  /// world-to-base transformation obtained through left foot
  pinocchio::SE3 m_oMff_rf;  /// world-to-base transformation obtained through right foot
  SE3 m_oMlfs;               /// transformation from world to left foot sole
  SE3 m_oMrfs;               /// transformation from world to right foot sole
  Vector7 m_oMlfs_xyzquat;
  Vector7 m_oMrfs_xyzquat;
  SE3 m_oMlfs_default_ref;  /// Default reference for left foot pose to use if no ref is pluged
  SE3 m_oMrfs_default_ref;  /// Default reference for right foot pose to use if no ref is pluged
  normal m_normal;          /// Normal distribution

  bool m_isFirstIterFlag;  /// flag to detect first iteration and initialise velocity filters

  SE3 m_sole_M_ftSens;  /// foot sole to F/T sensor transformation

  pinocchio::FrameIndex m_right_foot_id;
  pinocchio::FrameIndex m_left_foot_id;
  pinocchio::FrameIndex m_IMU_body_id;

  Eigen::VectorXd m_q_pin;  /// robot configuration according to pinocchio convention
  Eigen::VectorXd m_q_sot;  /// robot configuration according to SoT convention
  Eigen::VectorXd m_v_pin;  /// robot velocities according to pinocchio convention
  Eigen::VectorXd m_v_sot;  /// robot velocities according to SoT convention
  Matrix3 m_oRchest;        /// chest orientation in the world from angular fusion
  Matrix3 m_oRff;           /// base orientation in the world

  /* Filter buffers*/
  Vector3 m_last_vel;
  Vector3 m_last_DCvel;
  Vector3 m_last_DCacc;

};  // class BaseEstimator

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_free_flyer_locator_H__
